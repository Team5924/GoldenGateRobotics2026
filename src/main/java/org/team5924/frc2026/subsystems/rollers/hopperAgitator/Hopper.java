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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class Hopper extends SubsystemBase {
  public enum HopperState {

    // Hopper States: On is on; spit is for when we spit out from intake; off is off
    ON(new LoggedTunableNumber("HopperAgitatorOnVoltage", 0.0)),
    SPIT(new LoggedTunableNumber("HopperAgitatorSpitVoltage", 0.0)),
    OFF(new LoggedTunableNumber("HopperAgitatorOffVoltage", 0.0));

    private DoubleSupplier hopperVoltage;

    HopperState(LoggedTunableNumber hopperVoltage) {
      this.hopperVoltage = hopperVoltage;
    }
  }

  private final HopperIO io;

  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private HopperState goalState = HopperState.OFF;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setGoalState(HopperState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setHopperState(goalState);
  }
}
