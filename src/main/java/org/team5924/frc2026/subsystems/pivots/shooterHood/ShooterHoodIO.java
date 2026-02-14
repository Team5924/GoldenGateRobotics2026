/*
 * ShooterHoodIO.java
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

package org.team5924.frc2026.subsystems.pivots.shooterHood;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {
  @AutoLog
  public static class ShooterHoodIOInputs {
    public boolean shooterHoodMotorConnected = true;
    public double shooterHoodPositionRads = 0.0;
    public double shooterHoodVelocityRadsPerSec = 0.0;
    public double shooterHoodAppliedVoltage = 0.0;
    public double shooterHoodSupplyCurrentAmps = 0.0;
    public double shooterHoodTorqueCurrentAmps = 0.0;
    public double shooterHoodTempCelsius = 0.0;
  }

  /**
   * Updates the inputs object with the latest data from hardware
   *
   * @param inputs Inputs to update
   */
  public default void updateInputs(ShooterHoodIOInputs inputs) {}

  /**
   * Sets the subsystem motor to the specified voltage
   *
   * @param volts number of volts
   */
  public default void runVolts(double volts) {}

  public default void setPosition(double rads) {}

  /** stops the motor */
  default void stop() {}
}
