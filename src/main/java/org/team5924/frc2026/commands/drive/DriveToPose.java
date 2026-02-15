/*
 * DriveToPose.java
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.drive.Drive;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> target;
  private Supplier<Pose2d> robot = RobotState.getInstance()::getEstimatedPose;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private LoggedTunableNumber omegaFF = new LoggedTunableNumber("DriveToPose/omegaFF", 0.0);

  // PID Controllers
  private final LoggedTunableNumber driveKp = new LoggedTunableNumber("DriveToPose/driveKp", 2.0);
  private final LoggedTunableNumber driveKi = new LoggedTunableNumber("DriveToPose/driveKi", 0.0);
  private final LoggedTunableNumber driveKd = new LoggedTunableNumber("DriveToPose/driveKd", 0.0);
  private final LoggedTunableNumber thetaKp = new LoggedTunableNumber("DriveToPose/thetaKp", 4.0);
  private final LoggedTunableNumber thetaKi = new LoggedTunableNumber("DriveToPose/thetaKi", 0.0);
  private final LoggedTunableNumber thetaKd = new LoggedTunableNumber("DriveToPose/thetaKd", 0.2);

  private final PIDController drivePID =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get());
  private final PIDController thetaPID =
      new PIDController(thetaKp.get(), thetaKi.get(), thetaKd.get());

  // Clamp translational speed to a safe maximum (adjust to your robot's max m/s)
  private static final LoggedTunableNumber MAX_DRIVE_SPEED =
      new LoggedTunableNumber("DriveToPose/MAX_DRIVE_SPEED", 4.0); // m/s
  private static final LoggedTunableNumber MAX_TURN_SPEED =
      new LoggedTunableNumber("DriveToPose/MAX_TURN_SPEED", 6.0); // rad/s

  // Tolerances
  private static final LoggedTunableNumber DIST_TOL =
      new LoggedTunableNumber("DriveToPose/DIST_TOL", 0.05); // meters
  private static final LoggedTunableNumber ANGLE_TOL =
      new LoggedTunableNumber("DriveToPose/ANGLE_TOL", Math.toRadians(2));

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    addRequirements(drive);

    thetaPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public DriveToPose(Drive drive, Supplier<Pose2d> targetPose, Supplier<Pose2d> robot) {
    this(drive, targetPose);
    this.robot = robot;
  }

  public DriveToPose(
      Drive drive,
      Supplier<Pose2d> target,
      Supplier<Pose2d> robot,
      Supplier<Translation2d> linearFF) {
    this(drive, target, robot);
    this.linearFF = linearFF;
  }

  @Override
  public void initialize() {
    drivePID.reset();
    thetaPID.reset();
  }

  @Override
  public void execute() {

    Pose2d current = robot.get();
    Pose2d targetPose = target.get();

    Translation2d errorTranslation = targetPose.getTranslation().minus(current.getTranslation());

    double distanceError = errorTranslation.getNorm();

    double driveSpeed =
        MathUtil.clamp(
            -drivePID.calculate(distanceError, 0), -MAX_DRIVE_SPEED.get(), MAX_DRIVE_SPEED.get());

    double turnSpeed =
        MathUtil.clamp(
            thetaPID.calculate(
                current.getRotation().getRadians(), targetPose.getRotation().getRadians()),
            -MAX_TURN_SPEED.get(),
            MAX_TURN_SPEED.get());

    Rotation2d driveDirection = errorTranslation.getAngle();

    Translation2d linearVelocity = new Translation2d(driveSpeed, driveDirection);
    linearVelocity = linearVelocity.plus(linearFF.get());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX(),
            linearVelocity.getY(),
            turnSpeed + omegaFF.getAsDouble(),
            current.getRotation());
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    double dist = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    double angle = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

    return dist < DIST_TOL.get() && angle < ANGLE_TOL.get();
  }
}
