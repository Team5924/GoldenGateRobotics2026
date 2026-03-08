/*
 * LaunchCalculator.java
 */

/* 
 * Copyright (C) 2025-2026 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2026.util;

import static org.team5924.frc2026.util.LauncherConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.FieldConstants;
import org.team5924.frc2026.RobotState;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {
  private static LaunchCalculator instance;

  @Getter private double hoodAngleOffsetDeg = 0.0;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIODIC_SECONDS));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIODIC_SECONDS));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double turretRadians,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Presets
  public static final double hubPresetDistance = 0.96;
  public static final double towerPresetDistance = 2.5;
  public static final double trenchPresetDistance = 3.03;
  public static final double outpostPresetDistance = 4.84;
  public static final double passingPresetDistance = 7.0;
  public static final LaunchPreset passingPreset;
  public static final LaunchPreset hubPreset;
  public static final LaunchPreset towerPreset;
  public static final LaunchPreset trenchPreset;
  public static final LaunchPreset outpostPreset;
  public static final LaunchPreset hoodMinPreset =
      new LaunchPreset(
          new LoggedTunableNumber(
              "LaunchCalculator/Presets/HoodMin/HoodAngle",
              Units.radiansToDegrees(Constants.GeneralShooterHood.MIN_POSITION_RADS)),
          new LoggedTunableNumber("LaunchCalculator/Presets/HoodMin/FlywheelSpeed", 50));
  public static final LaunchPreset hoodMaxPreset =
      new LaunchPreset(
          new LoggedTunableNumber(
              "LaunchCalculator/Presets/HoodMax/HoodAngle",
              Units.radiansToDegrees(Constants.GeneralShooterHood.MIN_POSITION_RADS)),
          new LoggedTunableNumber("LaunchCalculator/Presets/HoodMax/FlywheelSpeed", 50));

  public static final LoggedTunableNumber passingIdleSpeed =
      new LoggedTunableNumber("LaunchCalculator/PassingIdleSpeed", 50);

  public static record LaunchPreset(
      LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  // Passing targets
  private static final double hubPassLine =
      FieldConstants.LinesHorizontal.rightBumpStart
          - Constants.TRACK_WIDTH_Y_METERS / 2.0; // left to right
  private static final double xPassTarget = Units.inchesToMeters(37);
  private static final double yPassTarget = Units.inchesToMeters(80);
  // Boxes of bad
  // Under tower
  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));

  // Behind the hubs
  private static final Bounds nearHubBound =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.neutralZoneNear + Units.inchesToMeters(65),
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);
  private static final Bounds farHubBound =
      new Bounds(
          FieldConstants.LinesVertical.oppAllianceZone,
          FieldConstants.fieldLength,
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);

  static {
    minDistance = 0.9;
    maxDistance = 4.9;
    passingMinDistance = 0.0;
    passingMaxDistance = 12.0;
    phaseDelay = 0.03;

    // if (true) {
    hoodAngleMap.put(0.96, Rotation2d.fromDegrees(10.0));
    hoodAngleMap.put(1.16, Rotation2d.fromDegrees(12.0));
    hoodAngleMap.put(1.58, Rotation2d.fromDegrees(14.0));
    hoodAngleMap.put(2.07, Rotation2d.fromDegrees(18.5));
    hoodAngleMap.put(2.37, Rotation2d.fromDegrees(22.0));
    hoodAngleMap.put(2.47, Rotation2d.fromDegrees(23.0));
    hoodAngleMap.put(2.70, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.94, Rotation2d.fromDegrees(25.0));
    hoodAngleMap.put(3.48, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.92, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(4.35, Rotation2d.fromDegrees(34.0));
    hoodAngleMap.put(4.84, Rotation2d.fromDegrees(38.0));

    flywheelSpeedMap.put(0.96, 150.0);
    flywheelSpeedMap.put(1.16, 155.0);
    flywheelSpeedMap.put(1.58, 160.0);
    flywheelSpeedMap.put(2.07, 165.0);
    flywheelSpeedMap.put(2.37, 170.0);
    flywheelSpeedMap.put(2.47, 170.0);
    flywheelSpeedMap.put(2.70, 170.0);
    flywheelSpeedMap.put(2.94, 175.0);
    flywheelSpeedMap.put(3.48, 175.0);
    flywheelSpeedMap.put(3.92, 180.0);
    flywheelSpeedMap.put(4.35, 185.0);
    flywheelSpeedMap.put(4.84, 190.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

    passingHoodAngleMap.put(5.46, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(6.62, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(7.80, Rotation2d.fromDegrees(38.0));

    passingFlywheelSpeedMap.put(5.46, 160.0);
    passingFlywheelSpeedMap.put(6.62, 180.0);
    passingFlywheelSpeedMap.put(7.80, 200.0);

    passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    passingTimeOfFlightMap.put(passingMaxDistance, 0.0);
    // } else {
    //   // Full field maps

    //   hoodAngleMap.put(minDistance, Rotation2d.fromDegrees(0.0));
    //   hoodAngleMap.put(maxDistance, Rotation2d.fromDegrees(0.0));

    //   flywheelSpeedMap.put(minDistance, 0.0);
    //   flywheelSpeedMap.put(maxDistance, 0.0);

    //   timeOfFlightMap.put(minDistance, 0.0);
    //   timeOfFlightMap.put(maxDistance, 0.0);

    //   passingHoodAngleMap.put(passingMinDistance, Rotation2d.fromDegrees(0.0));
    //   passingHoodAngleMap.put(passingMaxDistance, Rotation2d.fromDegrees(0.0));

    //   passingFlywheelSpeedMap.put(passingMinDistance, 0.0);
    //   passingFlywheelSpeedMap.put(passingMaxDistance, 0.0);

    //   passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    //   passingTimeOfFlightMap.put(passingMaxDistance, 0.0);
    // }

    passingPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Passing/HoodAngle",
                hoodAngleMap.get(passingPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Passing/FlywheelSpeed",
                flywheelSpeedMap.get(passingPresetDistance)));
    hubPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Hub/HoodAngle",
                hoodAngleMap.get(hubPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Hub/FlywheelSpeed",
                flywheelSpeedMap.get(hubPresetDistance)));
    towerPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Tower/HoodAngle",
                hoodAngleMap.get(towerPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Tower/FlywheelSpeed",
                flywheelSpeedMap.get(towerPresetDistance)));
    trenchPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Trench/HoodAngle",
                hoodAngleMap.get(trenchPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Trench/FlywheelSpeed",
                flywheelSpeedMap.get(trenchPresetDistance)));
    outpostPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Outpost/HoodAngle",
                hoodAngleMap.get(outpostPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Outpost/FlywheelSpeed",
                flywheelSpeedMap.get(outpostPresetDistance)));
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters(boolean isLeft) {
    Transform3d robotToLauncher = isLeft ? robotToLauncherLeft : robotToLauncherRight;
    boolean passing =
        AllianceFlipUtil.applyX(RobotState.getInstance().getEstimatedPose().getX())
            > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target =
        passing
            ? getPassingTarget()
            : AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d launcherPosition = estimatedPose.transformBy(robotToLauncher.toTransform2d());
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    // Calculate field relative launcher velocity
    var robotVelocity = RobotState.getInstance().getFieldSetpointVelocity();
    var robotAngle = RobotState.getInstance().getRotation();
    ChassisSpeeds launcherVelocity =
        DriverStation.isAutonomous()
            ? robotVelocity
            : GeomUtil.transformVelocity(
                robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    double turretRads = getTurretAngle(lookaheadPose, target, isLeft).getRadians();

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose.transformBy(robotToLauncher.toTransform2d().inverse());
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target, isLeft);

    // Calculate remaining parameters
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians()
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.LOOP_PERIODIC_SECONDS);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.LOOP_PERIODIC_SECONDS);
    lastDriveAngle = driveAngle;

    // Check if inside a box of bad
    var flippedPose = AllianceFlipUtil.apply(estimatedPose);
    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
    boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

    double flywheelVelocity =
        passing
            ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
            : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            outsideOfBadBoxes
                && lookaheadLauncherToTargetDistance >= (passing ? passingMinDistance : minDistance)
                && lookaheadLauncherToTargetDistance
                    <= (passing ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            hoodAngle + Units.degreesToRadians(hoodAngleOffsetDeg),
            hoodVelocity,
            flywheelVelocity,
            turretRads,
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

    return latestParameters;
  }

  private static Rotation2d getTurretAngle(
      Pose2d turretPose, Translation2d target, boolean isLeft) {

    Rotation2d turretToHub = target.minus(turretPose.getTranslation()).getAngle();
    Rotation2d turretRads = turretPose.getRotation().minus(turretToHub);
    return turretRads;
  }

  private static Rotation2d getDriveAngleWithLauncherOffset(
      Pose2d robotPose, Translation2d target, boolean isLeft) {
    Transform3d robotToLauncher = isLeft ? robotToLauncherLeft : robotToLauncherRight;

    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotToLauncher.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle =
        fieldToHubAngle.plus(hubAngle).plus(robotToLauncher.getRotation().toRotation2d());
    return driveAngle;
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public Translation2d getPassingTarget() {
    double flippedY = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose()).getY();
    boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

    // Check if we need to interpolate
    if (FieldConstants.fieldWidth - hubPassLine > flippedY && flippedY > hubPassLine) {
      double interpolateZoneAmount =
          ((mirror ? FieldConstants.fieldWidth - flippedY : flippedY) - hubPassLine)
              / (FieldConstants.LinesHorizontal.center - hubPassLine);
      var unflippedPoseY =
          mirror
              ? FieldConstants.fieldWidth
                  - MathUtil.interpolate(yPassTarget, passingMinDistance, interpolateZoneAmount)
              : MathUtil.interpolate(yPassTarget, passingMinDistance, interpolateZoneAmount);
      Translation2d flippedGoalTranslation =
          AllianceFlipUtil.apply(new Translation2d(xPassTarget, unflippedPoseY));
      return flippedGoalTranslation;
    }

    // Fixed passing target
    Translation2d flippedGoalTranslation =
        AllianceFlipUtil.apply(
            new Translation2d(
                xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget));

    return flippedGoalTranslation;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @param forceBlue Always use the blue hub target
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(
      Translation2d robotTranslation, boolean forceBlue, boolean isLeft) {
    // Calculate target
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }

    return new Pose2d(
        robotTranslation,
        getDriveAngleWithLauncherOffset(robotTranslation.toPose2d(), target, isLeft));
  }

  /** Adjusts the hood angle offset up or down the specified amount. */
  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }
}
