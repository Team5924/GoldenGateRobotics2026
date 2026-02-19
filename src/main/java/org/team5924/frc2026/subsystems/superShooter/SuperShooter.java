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
import lombok.Setter;

import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRoller;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRoller.ShooterRollerState;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.turret.Turret;

public class SuperShooter extends SubsystemBase {
  @Getter private final ShooterRoller roller;
  @Getter private final ShooterHood hood;
  @Getter private final Turret turret;

  public enum ShooterState {
    OFF(),
    MANUAL(),
    AUTO_SHOOTING(),
    BUMPER_SHOOTING(),
    NEUTRAL_SHUFFLING(),
    OPPONENT_SHUFFLING();

    ShooterState() {}
  }

  @Getter private ShooterState goalState = ShooterState.OFF;

  public void runRollerVolts(double volts) {
    roller.runVolts(volts);
  }

  public void runHoodVolts(double volts) {
    hood.runVolts(volts);
  }

  public void runTurretVolts(double volts) {
    turret.tryRunVolts(volts);
  }

  public void setHoodPosition(double rads) {
    hood.setPosition(rads);
  }

  public void setRollerInput(double input) {
    roller.setInput(input);
  }

  public void setHoodInputs(double input) {
    hood.setInput(input);
  }

  public void setTurretInputs(double input) {
    turret.setInput(input);
  }

  public void setGoalState(ShooterState goalState) {
    this.goalState = goalState;
  }

  public SuperShooter(ShooterRoller roller, ShooterHood hood, Turret turret) {
    this.roller = roller;
    this.hood = hood;
    this.turret = turret;
  }

  @Override
  public void periodic() {
    // switch (goalState) {
    //   case OFF:
    //     roller.setGoalState(ShooterRollerState.OFF);
    //     hood.setGoalState(ShooterHoodState.OFF);
    //     break;
    //   case MANUAL:
    //     hood.setGoalState(ShooterHoodState.BUMPER_SHOOTING);
    //     break;
    //   case AUTO_SHOOTING:
    //     roller.setGoalState(ShooterRollerState.AUTO_SHOOTING);
    //     hood.setGoalState(ShooterHoodState.AUTO_SHOOTING);
    //     break;
    //   case BUMPER_SHOOTING:
    //     roller.setGoalState(ShooterRollerState.BUMPER_SHOOTING);
    //     hood.setGoalState(ShooterHoodState.BUMPER_SHOOTING);
    //     break;
    //   case NEUTRAL_SHUFFLING:
    //     roller.setGoalState(ShooterRollerState.NEUTRAL_SHUFFLING);
    //     hood.setGoalState(ShooterHoodState.NEUTRAL_SHUFFLING);
    //     break;
    //   case OPPONENT_SHUFFLING:
    //     roller.setGoalState(ShooterRollerState.OPPONENT_SHUFFLING);
    //     hood.setGoalState(ShooterHoodState.OPPONENT_SHUFFLING);
    //     break;
    // }
  }
}
