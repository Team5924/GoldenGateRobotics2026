/*
 * ShooterRoller.java
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

package org.team5924.frc2026.subsystems.rollers.shooterRoller;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class ShooterRoller
    extends GenericRollerSystem<
        ShooterRoller.ShooterRollerState,
        ShooterRollerIOInputs,
        ShooterRollerIO,
        ShooterRollerIOInputsAutoLogged> {

  // private static DigitalInput beamBreak;

  @RequiredArgsConstructor
  @Getter
  public enum ShooterRollerState implements VoltageState { // TODO: update voltage values
    OFF(new LoggedTunableNumber("ShooterRoller/Off", 0)),
    AUTO_SHOOTING(new LoggedTunableNumber("ShooterRoller/AutoShooting", -1)),
    BUMPER_SHOOTING(new LoggedTunableNumber("ShooterRoller/BumperShooting", 8)),
    NEUTRAL_SHUFFLING(new LoggedTunableNumber("ShooterRoller/NeutralShuffling", -1)),
    OPPONENT_SHUFFLING(new LoggedTunableNumber("ShooterRoller/OpponentShuffling", -1));

    private final LoggedTunableNumber voltageSupplier;
  }

  private ShooterRollerState goalState = ShooterRollerState.OFF;

  public ShooterRoller(ShooterRollerIO io) {
    super("ShooterRoller", io, new ShooterRollerIOInputsAutoLogged());
  }

  public void setGoalState(ShooterRollerState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setShooterRollerState(goalState);
  }

  // public static boolean isGamePieceDetected() {
  //   return beamBreak != null && beamBreak.get();
  // }
}
