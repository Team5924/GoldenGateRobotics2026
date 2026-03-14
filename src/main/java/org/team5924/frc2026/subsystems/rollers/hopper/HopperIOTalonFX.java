/*
 * HopperIOTalonFX.java
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

package org.team5924.frc2026.subsystems.rollers.hopper;

import org.team5924.frc2026.Constants.Hopper;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerIOTalonFX;

public class HopperIOTalonFX extends GenericRollerIOTalonFX implements HopperIO {
  public HopperIOTalonFX() {
    super(Hopper.CAN_ID, Hopper.BUS, Hopper.CONFIG, Hopper.MOTOR_TO_MECHANISM);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
  }
}
