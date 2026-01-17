/*
 * HopperIO.java
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

import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIO;

public interface HopperIO extends GenericRollerSystemIO {
  @AutoLog
  public static class HopperIOInputs extends GenericRollerSystemIOInputs {
    // public boolean isFull = false;
  }

  // Updates motor (and beam break if we add) inputs
  public default void updateInputs(HopperIOInputs inputs) {}

  // Runs Motor at inputted volts
  default void runVolts(double volts) {}
}
