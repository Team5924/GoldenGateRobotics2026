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

  //Hopper Beam Breaks
  private final BeamBreakIO beamBreakIO_1;
  private final BeamBreakIO beamBreakIO_2;
  private final BeamBreakIO beamBreakIO_3;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs_1 = new BeamBreakIOInputsAutoLogged();
  private final BeamBreakIOInputsAutoLogged beamBreakInputs_2 = new BeamBreakIOInputsAutoLogged();
  private final BeamBreakIOInputsAutoLogged beamBreakInputs_3 = new BeamBreakIOInputsAutoLogged();

  public Hopper(HopperIO io, BeamBreakIO beamBreakIO_1, BeamBreakIO beamBreakIO_2, BeamBreakIO beamBreakIO_3) {
    super("Hopper", io, new HopperIOInputsAutoLogged());
    this.beamBreakIO_1 = beamBreakIO_1;
    this.beamBreakIO_2 = beamBreakIO_2;
    this.beamBreakIO_3 = beamBreakIO_3;
  }

  public void setGoalState(HopperState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setHopperState(goalState);
  }

  @Override
  public void periodic() {
    beamBreakIO_1.updateInputs(beamBreakInputs_1);
    beamBreakIO_2.updateInputs(beamBreakInputs_2);
    beamBreakIO_3.updateInputs(beamBreakInputs_3);
    inputs.isFull = beamBreakInputs_1.data.broken() && beamBreakInputs_2.data.broken() && beamBreakInputs_3.data.broken();
    super.periodic();
  }
}
