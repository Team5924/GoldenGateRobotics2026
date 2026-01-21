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

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class ShooterRoller extends GenericRollerSystem<ShooterRoller.ShooterRollerState> {
  @RequiredArgsConstructor
  @Getter
  public enum ShooterRollerState implements VoltageState {
    SHOOTING(new LoggedTunableNumber("ShooterRoller/Shooting", 12.0)),
    SHUFFLING(new LoggedTunableNumber("ShooterRoller/Shuffling", 12.0)),
    OFF(new LoggedTunableNumber("ShooterRoller/Off", 0.0));

    private final DoubleSupplier voltageSupplier;
  }

  private ShooterRollerState goalState = ShooterRollerState.OFF;

  public ShooterRoller(ShooterRollerIO inputs) {
    super("ShooterRoller", inputs);
  }

  @Override
  public void periodic() {
    ((ShooterRollerIO) io).runVolts(goalState.getVoltageSupplier().getAsDouble());
    super.periodic();
  }

  public void setGoalState(ShooterRollerState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setShooterRollerState(goalState);
  }
}
