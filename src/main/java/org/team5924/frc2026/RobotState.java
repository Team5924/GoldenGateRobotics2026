/*
 * RobotState.java
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

package org.team5924.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.subsystems.SuperShooter.ShooterState;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot.IntakePivotState;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.rollers.hopper.Hopper.HopperState;
import org.team5924.frc2026.subsystems.rollers.indexer.Indexer.IndexerState;
import org.team5924.frc2026.subsystems.rollers.intake.Intake.IntakeState;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRoller.ShooterRollerState;
import org.team5924.frc2026.subsystems.turret.Turret.TurretState;

@Getter
public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public static double getTime() {
    return ((double) Logger.getTimestamp() * 1.0E-6);
  }

  Rotation2d gyroOffset = Rotation2d.kZero;

  // Pose Estimation Members
  @AutoLogOutput(key = "RobotState/OdometryPose")
  @Getter
  @Setter
  private Pose2d odometryPose = new Pose2d();

  @Getter @Setter @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    odometryPose = pose;
    estimatedPose = pose;
  }

  @Getter @Setter private Rotation2d yawPosition = new Rotation2d();
  @Getter @Setter private double yawVelocityRadPerSec = 0.0;

  /* ### Intake ### */
  @Getter @Setter private IntakeState intakeState = IntakeState.OFF;

  /* ### Intake ### */
  @Getter @Setter private IntakePivotState intakePivotState = IntakePivotState.OFF;

  /* ### Hopper ### */
  @Getter @Setter private HopperState hopperState = HopperState.OFF;

  /* ### Indexer ### */
  @Getter @Setter private IndexerState indexerState = IndexerState.OFF;

  /*### Shooter ### */
  @Getter @Setter private ShooterState leftShooterState = ShooterState.OFF;
  @Getter @Setter private ShooterState rightShooterState = ShooterState.OFF;

  // Turret
  @Getter @Setter private TurretState leftTurretState = TurretState.OFF;
  @Getter @Setter private TurretState rightTurretState = TurretState.OFF;

  /*### Shooter Hood ### */
  @Getter @Setter private ShooterHoodState leftShooterHoodState = ShooterHoodState.OFF;
  @Getter @Setter private ShooterHoodState rightShooterHoodState = ShooterHoodState.OFF;

  /*### Shooter Roller ### */
  @Getter @Setter private ShooterRollerState leftShooterRollerState = ShooterRollerState.OFF;
  @Getter @Setter private ShooterRollerState rightShooterRollerState = ShooterRollerState.OFF;
}
