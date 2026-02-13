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

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.beamBreak.BeamBreakIO;
import org.team5924.frc2026.subsystems.beamBreak.BeamBreakIOInputsAutoLogged;
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
    OFF(() -> 0.0);

    private final DoubleSupplier voltageSupplier;
  }

  private HopperState goalState = HopperState.OFF;

  // Hopper Beam Breaks
  private final BeamBreakIO[] beamBreakIOs;
  private final BeamBreakIOInputsAutoLogged[] beamBreakInputs;
  public Hopper(
      HopperIO io,
      BeamBreakIO[] beamBreakIOs) {
    super("Hopper", io, new HopperIOInputsAutoLogged());
    this.beamBreakIOs = beamBreakIOs;
    this.beamBreakInputs = new BeamBreakIOInputsAutoLogged[beamBreakIOs.length];
    for (int i = 0; i < beamBreakIOs.length; i++) {
      beamBreakInputs[i] = new BeamBreakIOInputsAutoLogged();
    }
  }

  public void setGoalState(HopperState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setHopperState(goalState);
  }

  @Override
  public void periodic() {
    boolean allBroken = true;
    for (int i = 0; i < beamBreakIOs.length; i++) {
      beamBreakIOs[i].updateInputs(beamBreakInputs[i]);
      allBroken &= beamBreakInputs[i].broken;
    }
    inputs.isFull = allBroken;
    super.periodic();
  }
}
