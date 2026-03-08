/*
 * AutoScoreCommands.java
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.rollers.shooterFlywheel.ShooterFlywheel;
import org.team5924.frc2026.subsystems.rollers.shooterFlywheel.ShooterFlywheel.ShooterFlywheelState;
import org.team5924.frc2026.subsystems.turret.Turret;
import org.team5924.frc2026.subsystems.turret.Turret.TurretState;
import org.team5924.frc2026.util.LaunchCalculator;
import org.team5924.frc2026.util.LaunchCalculator.LaunchingParameters;

public class AutoScoreCommands {
  // TODO Make and auto score program
  // public static Command autoScore(
  //   Drive drive,
  //   SuperShooter shooter,
  //   DoubleSupplier driverX,
  //   DoubleSupplier driverY,
  //   DoubleSupplier driverOmega,
  //   Command joystickDrive,
  //   Supplier<Command> controllerRumble,
  //   BooleanSupplier trigger,
  //   BooleanSupplier disableReefAutoAlign,
  //   BooleanSupplier manualEject) {
  //   }
  private AutoScoreCommands() {}

  public static Command runTrackTargetCommand(
      Turret turret, ShooterHood shooterHood, ShooterFlywheel shooterFlywheel, boolean isLeft) {
    return Commands.run(
        () -> {
          LaunchingParameters launchParams = LaunchCalculator.getInstance().getParameters(isLeft);

          turret.setAutoInput(launchParams.turretRadians());
          shooterHood.setAutoInput(launchParams.hoodAngle());
          shooterFlywheel.setAutoInput(launchParams.flywheelSpeed());

          turret.setGoalState(TurretState.AUTO);
          shooterHood.setGoalState(ShooterHoodState.AUTO);
          shooterFlywheel.setGoalState(ShooterFlywheelState.AUTO);
        },
        turret,
        shooterHood,
        shooterFlywheel);
  }
}
