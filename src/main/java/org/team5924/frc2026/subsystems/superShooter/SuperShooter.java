/*
 * SuperShooter.java
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

package org.team5924.frc2026.subsystems.superShooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRoller;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRoller.ShooterRollerState;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class SuperShooter extends SubsystemBase {
  private final ShooterRoller roller;
  private final ShooterHood hood;

  public enum ShooterState {
    OFF(new LoggedTunableNumber("Shooter/Off", Math.toRadians(0))),
    AUTO_SHOOTING(new LoggedTunableNumber("Shooter/Auto_Shooting", 0)),
    BUMPER_SHOOTING(new LoggedTunableNumber("Shooter/Bumper_Shooting", Math.toRadians(90))),
    NEUTRAL_SHUFFLING(new LoggedTunableNumber("Shooter/Neutral_Shuffling", Math.toRadians(90))),
    OPPONENT_SHUFFLING(new LoggedTunableNumber("Shooter/Opponent_Shuffling", Math.toRadians(90))),
    HOOD_MOVING(new LoggedTunableNumber("Shooter/Hood_Moving", -1));

    private final LoggedTunableNumber rads;

    ShooterState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  private ShooterState state = ShooterState.OFF;

  @Getter private ShooterState goalState;

  public SuperShooter(ShooterRoller roller, ShooterHood hood) {
    this.roller = roller;
    this.hood = hood;
  }

  @Override
  public void periodic() {
    switch (state) {
      case OFF:
        roller.setGoalState(ShooterRollerState.OFF);
        hood.setGoalState(ShooterHoodState.OFF);
      case AUTO_SHOOTING:
        roller.setGoalState(ShooterRollerState.SHOOTING);
        hood.setGoalState(ShooterHoodState.AUTO_SHOOTING);
      case BUMPER_SHOOTING:
        roller.setGoalState(ShooterRollerState.SHOOTING);
        hood.setGoalState(ShooterHoodState.BUMPER_SHOOTING);
    case NEUTRAL_SHUFFLING:
        roller.setGoalState(ShooterRollerState.SHOOTING);
        hood.setGoalState(ShooterHoodState.NEUTRAL_SHUFFLING);
    case OPPONENT_SHUFFLING:
        roller.setGoalState(ShooterRollerState.SHOOTING);
        hood.setGoalState(ShooterHoodState.OPPONENT_SHUFFLING);
    case HOOD_MOVING:
        roller.setGoalState(ShooterRollerState.OFF);
        hood.setGoalState(ShooterHoodState.MOVING);
    }
  }
}
