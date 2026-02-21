/*
 * AutoBuilder.java
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

package org.team5924.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.Robot;
import org.team5924.frc2026.subsystems.SuperShooter;
import org.team5924.frc2026.subsystems.SuperShooter.ShooterState;
import org.team5924.frc2026.subsystems.drive.Drive;
import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.rollers.intake.Intake.IntakeState;

@RequiredArgsConstructor
public class AutoBuilder {

  private final Drive drive;
  private final SuperShooter shooter;
  // private final Climb climb;
  private final Intake intake;

  // Left, Mid, Right 1-5
  private static Supplier<String> startingPositionSupplier;

  public static void setStartingPosition(Supplier<String> supplier) {
    if (supplier == null)
      throw new IllegalArgumentException("startingPositionSupplier must not be null");
    startingPositionSupplier = supplier;
  }

  public Command scoreAndClimbAuto() {
    if (startingPositionSupplier == null) {
      throw new IllegalStateException(
          "setStartingPosition() must be called before building auto commands");
    }
    return Commands.sequence(
        startToHub(startingPositionSupplier.get()),
        Commands.run(() -> shooter.setGoalState(ShooterState.AUTO_SHOOTING), shooter)
            .withTimeout(1.0),
        Commands.runOnce(() -> shooter.setGoalState(ShooterState.OFF), shooter),
        Robot.mAutoFactory.trajectoryCmd("HubToClimb")
        // Commands.run(() -> climb.setGoalState(ClimbState.L1_CLIMB), climb)
        );
  }

  public Command scorePickupAndClimbAuto() {
    if (startingPositionSupplier == null) {
      throw new IllegalStateException(
          "setStartingPosition() must be called before building auto commands");
    }
    return Commands.sequence(
        startToHub(startingPositionSupplier.get()),
        Commands.run(() -> shooter.setGoalState(ShooterState.AUTO_SHOOTING), shooter)
            .withTimeout(1.0),
        Commands.runOnce(() -> shooter.setGoalState(ShooterState.OFF), shooter),
        Robot.mAutoFactory.trajectoryCmd("HubToDepot"),
        Commands.deadline(
            Robot.mAutoFactory.trajectoryCmd("DepotIntake"),
            Commands.run(() -> intake.setGoalState(IntakeState.INTAKE), intake)),
        Commands.runOnce(() -> intake.setGoalState(IntakeState.OFF), intake),
        Robot.mAutoFactory.trajectoryCmd("DepotToHub"),
        Commands.run(() -> shooter.setGoalState(ShooterState.AUTO_SHOOTING), shooter)
            .withTimeout(1.0),
        Commands.runOnce(() -> shooter.setGoalState(ShooterState.OFF), shooter),
        Robot.mAutoFactory.trajectoryCmd("HubToClimb")
        // Commands.run(() -> climb.setGoalState(ClimbState.L1_CLIMB), climb)
        );
  }

  private Command startToHub(String startingPosition) {
    if ("Mid".equals(startingPosition)) {
      return Commands.none();
    } else {
      return Commands.sequence(
          Robot.mAutoFactory.resetOdometry(startingPosition + "StartToHub"),
          Robot.mAutoFactory.trajectoryCmd(startingPosition + "StartToHub"));
    }
  }
}
