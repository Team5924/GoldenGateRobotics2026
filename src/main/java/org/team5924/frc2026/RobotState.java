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
import org.team5924.frc2026.subsystems.exampleSystem.ExampleSystem.ExampleSystemState;
import org.team5924.frc2026.subsystems.rollers.exampleRoller.ExampleRoller.ExampleRollerState;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRoller.ShooterRollerState;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter.ShooterState;

@Getter
public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  @AutoLogOutput(key = "RobotState/OdometryPose")
  @Getter
  @Setter
  private Pose2d odometryPose = new Pose2d();

  @Getter @Setter private Rotation2d yawPosition = new Rotation2d();
  @Getter @Setter private double yawVelocityRadPerSec = 0.0;

  /* ### Example Subsystem ### */
  @Getter @Setter private ExampleSystemState exampleSystemState = ExampleSystemState.STOW;

  /*### Shooter ### */
  @Getter @Setter private ShooterState shooterState = ShooterState.OFF;

  /*### Shooter Hood ### */
  @Getter @Setter private ShooterHoodState shooterHoodState = ShooterHoodState.OFF;

  /* ### Example Roller ### */
  @Getter @Setter private ExampleRollerState exampleRollerState = ExampleRollerState.IDLE;

  /*### Shooter Roller ### */
  @Getter @Setter private ShooterRollerState shooterRollerState = ShooterRollerState.OFF;
}
