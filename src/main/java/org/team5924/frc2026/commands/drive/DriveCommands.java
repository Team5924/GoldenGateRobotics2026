/*
 * DriveCommands.java
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

package org.team5924.frc2026.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.FieldConstants;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.generated.TunerConstants;
import org.team5924.frc2026.subsystems.drive.Drive;
import org.team5924.frc2026.util.AllianceFlipUtil;
import org.team5924.frc2026.util.GeomUtil;
import org.team5924.frc2026.util.LaunchCalculator;
import org.team5924.frc2026.util.LauncherConstants;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static final LoggedTunableNumber driveLaunchKp =
      new LoggedTunableNumber("DriveCommands/Launching/kP", 8.0);
  private static final LoggedTunableNumber driveLaunchKd =
      new LoggedTunableNumber("DriveCommands/Launching/kD", 0.5);
  private static final LoggedTunableNumber driveYawLaunchToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Launching/YawToleranceDeg", 5.0);
  private static final LoggedTunableNumber driveLaunchMaxPolarVelocityRadPerSec =
      new LoggedTunableNumber("DriveCommands/Launching/MaxPolarVelocityRadPerSec", 0.6);
  private static final LoggedTunableNumber driveLauncherCORMinErrorDeg =
      new LoggedTunableNumber("DriveCommands/Launching/DriveLauncherCORMinErrorDeg", 15.0);
  private static final LoggedTunableNumber driveLauncherCORMaxErrorDeg =
      new LoggedTunableNumber("DriveCommands/Launching/DriveLauncherCORMaxErrorDeg", 30.0);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command driveFacingHub(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () ->
            AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                .minus(drive.getPose().getTranslation())
                .getAngle());
  }

  public static Command joystickDriveWhileLaunching(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // Create command
    return Commands.run(
        () -> {
          // Run PID controller
          final var parameters = LaunchCalculator.getInstance().getParameters();
          double omegaOutput =
              parameters.driveVelocity()
                  + (parameters
                          .driveAngle()
                          .minus(RobotState.getInstance().getRotation())
                          .getRadians()
                      * driveLaunchKp.get())
                  + ((parameters.driveVelocity()
                          - RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond)
                      * driveLaunchKd.get());

          // Calculate speeds
          Translation2d fieldRelativeLinearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
          if (AllianceFlipUtil.shouldFlip()) {
            fieldRelativeLinearVelocity = fieldRelativeLinearVelocity.times(-1.0);
          }

          // Only limit if launching, not passing
          if (!LaunchCalculator.getInstance().getParameters().passing()) {
            // Calculate max linear velocity magnitude based on the max polar velocity
            double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
            double robotAngle =
                Math.abs(
                    AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                        .minus(RobotState.getInstance().getOdometryPose().getTranslation())
                        .getAngle()
                        .minus(fieldRelativeLinearVelocity.getAngle())
                        .getRadians());
            double robotHubDistance =
                LaunchCalculator.getInstance().getParameters().distanceNoLookahead();
            double hubAngle =
                driveLaunchMaxPolarVelocityRadPerSec.get()
                    * LaunchCalculator.getInstance().getNaiveTOF(robotHubDistance);
            double lookaheadAngle = Math.PI - robotAngle - hubAngle;

            // Calculate limit if triangle is valid (otherwise no limit)
            if (lookaheadAngle > 0) {
              double robotLookaheadDistance =
                  robotHubDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
              maxLinearVelocityMagnitude =
                  robotLookaheadDistance
                      / LaunchCalculator.getInstance().getNaiveTOF(robotHubDistance);
            }

            // Apply limit to velocity
            if (fieldRelativeLinearVelocity.getNorm() > maxLinearVelocityMagnitude) {
              fieldRelativeLinearVelocity =
                  fieldRelativeLinearVelocity.times(
                      maxLinearVelocityMagnitude / fieldRelativeLinearVelocity.getNorm());
            }
          }

          if (driveLauncherCORMaxErrorDeg.get() < driveLauncherCORMinErrorDeg.get()) return;

          // Apply chassis speeds
          double corScalar =
              MathUtil.clamp(
                  (Math.abs(
                              parameters
                                  .driveAngle()
                                  .minus(RobotState.getInstance().getRotation())
                                  .getDegrees())
                          - driveLauncherCORMinErrorDeg.get())
                      / (driveLauncherCORMaxErrorDeg.get() - driveLauncherCORMinErrorDeg.get()),
                  0.0,
                  1.0);
          Translation2d launcherToRobot =
              LauncherConstants.robotToLauncherCenter
                  .getTranslation()
                  .toTranslation2d()
                  .unaryMinus();
          ChassisSpeeds fieldRelativeSpeedsWithOffset =
              GeomUtil.transformVelocity(
                  new ChassisSpeeds(
                      fieldRelativeLinearVelocity.getX(),
                      fieldRelativeLinearVelocity.getY(),
                      omegaOutput),
                  launcherToRobot.times(1.0 - corScalar),
                  RobotState.getInstance().getRotation());
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeSpeedsWithOffset, RobotState.getInstance().getRotation()));

          // Override robot setpoint speeds published by drive. We run our calculations using the
          // speeds that will ultimately be applied once we are using the full robot-to-launcher
          // transform. This prevents the setpoint from changing due to the shifting COR of the
          // robot.
          ChassisSpeeds fieldRelativeSpeedsWithFullOffset =
              GeomUtil.transformVelocity(
                  new ChassisSpeeds(
                      fieldRelativeLinearVelocity.getX(),
                      fieldRelativeLinearVelocity.getY(),
                      omegaOutput),
                  launcherToRobot,
                  RobotState.getInstance().getRotation());
          RobotState.getInstance()
              .setRobotSetpointVelocity(
                  ChassisSpeeds.discretize(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          fieldRelativeSpeedsWithFullOffset,
                          RobotState.getInstance().getRotation()),
                      Constants.LOOP_PERIODIC_SECONDS));

          // Log data
          Logger.recordOutput(
              "DriveCommands/Launching/SetpointPose",
              new Pose2d(
                  RobotState.getInstance().getOdometryPose().getTranslation(),
                  parameters.driveAngle()));
          Logger.recordOutput("DriveCommands/Launching/AtGoalTolerance", atLaunchGoal());
          Logger.recordOutput(
              "DriveCommands/Launching/ErrorPosition",
              parameters.driveAngle().minus(RobotState.getInstance().getRotation()));
          Logger.recordOutput(
              "DriveCommands/Launching/ErrorVelocityRadPerSec",
              parameters.driveVelocity()
                  - RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond);
          Logger.recordOutput(
              "DriveCommands/Launching/MeasuredPosition", RobotState.getInstance().getRotation());
          Logger.recordOutput(
              "DriveCommands/Launching/MeasuredVelocityRadPerSec",
              RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond);
          Logger.recordOutput("DriveCommands/Launching/SetpointPosition", parameters.driveAngle());
          Logger.recordOutput(
              "DriveCommands/Launching/SetpointVelocityRadPerSec", parameters.driveVelocity());
        },
        drive);
  }

  public static boolean atLaunchGoal() {
    return DriverStation.isEnabled()
        && Math.abs(
                RobotState.getInstance()
                    .getRotation()
                    .minus(LaunchCalculator.getInstance().getParameters().driveAngle())
                    .getRadians())
            <= Units.degreesToRadians(driveYawLaunchToleranceDeg.get());
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
