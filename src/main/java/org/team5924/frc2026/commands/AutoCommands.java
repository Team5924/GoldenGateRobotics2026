/*
 * AutoCommands.java
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
import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.rollers.intake.Intake.IntakeState;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter.ShooterState;

public class AutoCommands {
  public static Command intake(Intake intake) {
    return Commands.run(() -> intake.setGoalState(IntakeState.INTAKE), intake);
  }

  //TODO: this command will put shooter into place for shooting, so it needs turret and rotation data

  // public static Command getShooterReady(SuperShooter shooter) {
  //     return Commands.run(() -> shooter.setGoalState(ShooterState.AUTO_SHOOTING), shooter);
  // }

  public static Command score(SuperShooter shooter) {
    return Commands.run(() -> shooter.setGoalState(ShooterState.AUTO_SHOOTING), shooter);
  }

  // no climb subsystem yet

  // public static Command climb(Climb climb) {
  //     return Commands.run(() -> climb.setGoalState(ClimbState.L1_CLIMB), climb);
  // }
}
