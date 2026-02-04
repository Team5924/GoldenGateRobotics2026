/*
 * ShooterCommands.java
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter.ShooterState;

public class ShooterCommands {

  private ShooterCommands() {}

  public static Command manualControl(
      SuperShooter shooter, DoubleSupplier hoodSupplier, DoubleSupplier rollerSupplier) {
    return Commands.run(
        () -> {
          shooter.runRollerVolts(hoodSupplier.getAsDouble());
          shooter.runHoodVolts(hoodSupplier.getAsDouble());
        },
        shooter);
  }

  public static ShooterState determineStateFromPosition() {
    double x = RobotState.getInstance().getOdometryPose().getX();
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get().equals(Alliance.Red);

    if (x <= Constants.Field.BLUE_HUB_X) { // Blue alliance zone
      return (isFlipped ? ShooterState.OPPONENT_SHUFFLING : ShooterState.AUTO_SHOOTING);
    } else if (x < Constants.Field.RED_HUB_X) { // Neutral zone
      return ShooterState.NEUTRAL_SHUFFLING;
    } else { // Red alliance zone
      return (isFlipped ? ShooterState.AUTO_SHOOTING : ShooterState.OPPONENT_SHUFFLING);
    }
  }

  public static Command autoSetState(SuperShooter shooter) {
    return Commands.run(() -> shooter.setGoalState(determineStateFromPosition()), shooter);
  }
}
