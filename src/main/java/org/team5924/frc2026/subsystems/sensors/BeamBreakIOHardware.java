/*
 * BeamBreakIOHardware.java
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

package org.team5924.frc2026.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOHardware implements BeamBreakIO {
  private final DigitalInput beamBreak;

  public BeamBreakIOHardware(int id) {
    beamBreak = new DigitalInput(id);
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.broken = beamBreak.get();
  }
}
