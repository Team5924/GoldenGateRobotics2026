/*
 * HopperKrakenFOC.java
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

// import edu.wpi.first.wpilibj.DigitalInput;
import org.team5924.frc2026.Constants.Hopper;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIOKrakenFOC;

public class HopperKrakenFOC extends GenericRollerSystemIOKrakenFOC<HopperIOInputs>
    implements HopperIO {
  /* Still undecided whether add a beambreak for if hopper is full*/

  // private static final DigitalInput fullBeamBreakSensor;

  public HopperKrakenFOC() {
    super(Hopper.CAN_ID, Hopper.BUS, Hopper.CONFIG, Hopper.REDUCTION);

    // fullBeamBreakSensor = Constants.HOPPER_BEAM_BREAK_ID
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    super.updateInputs(inputs);
    // inputs.isFull = fullBeamBreakSensor.get();
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
  }
}
