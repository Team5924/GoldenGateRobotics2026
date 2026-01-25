/*
 * Hopper.java
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

package org.team5924.frc2026.subsystems.rollers.hopperAgitator;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class Hopper
    extends GenericRollerSystem<
        Hopper.HopperState, HopperIOInputs, HopperIO, HopperIOInputsAutoLogged> {

  @RequiredArgsConstructor
  @Getter
  public enum HopperState implements VoltageState {
    ON(new LoggedTunableNumber("HopperAgitatorOnVoltage", 0.0)),
    SPIT(new LoggedTunableNumber("HopperAgitatorSpitVoltage", 0.0)),
    OFF(new LoggedTunableNumber("HopperAgitatorOffVoltage", 0.0));

    private final DoubleSupplier voltageSupplier;
  }

  private HopperState goalState = HopperState.OFF;

  public Hopper(HopperIO io) {
    super("Hopper", io, new HopperIOInputsAutoLogged());
  }

  public void setGoalState(HopperState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setHopperState(goalState);
  }
}
