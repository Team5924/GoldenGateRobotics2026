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

package org.team5924.frc2026.commands.shooter;

import java.util.function.DoubleSupplier;

import org.team5924.frc2026.subsystems.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter;
import org.team5924.frc2026.subsystems.turret.Turret;
import org.team5924.frc2026.subsystems.turret.Turret.TurretState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {

  private ShooterCommands() {}

  public static Command manualControl(
      SuperShooter shooter,
      DoubleSupplier hoodSupplier,
      DoubleSupplier rollerSupplier) {
    return Commands.run(
      () -> {
        shooter.runRollerVolts(hoodSupplier.getAsDouble());
        shooter.runHoodVolts(hoodSupplier.getAsDouble());
      },
      shooter
    );
  }

  public static Command manualShooterHood(
      ShooterHood hood,
      DoubleSupplier inputSupplier) {
    return Commands.run(
      () -> {
        hood.setGoalState(ShooterHoodState.MANUAL);
        hood.setInput(inputSupplier.getAsDouble());
      },
      hood
    );
  }

  public static Command manualTurret(
      Turret turret,
      DoubleSupplier inputSupplier) {
    return Commands.run(
      () -> {
        turret.setGoalState(TurretState.MANUAL);
        turret.setInput(inputSupplier.getAsDouble());
      },
      turret
    );
  }

  // public 
}
