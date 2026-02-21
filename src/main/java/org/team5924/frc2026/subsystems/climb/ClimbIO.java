/*
 * ClimbIO.java
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

package org.team5924.frc2026.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public boolean climbMotorConnected = true;
    public double climbPositionRads = 0.0;
    public double climbPositionCancoder = 0.0;
    public double climbVelocityRadsPerSec = 0.0;
    public double climbAppliedVoltage = 0.0;
    public double climbSupplyCurrentAmps = 0.0;
    public double climbTorqueCurrentAmps = 0.0;
    public double climbTempCelsius = 0.0;

    public boolean cancoderConnected = true;
    public double cancoderAbsolutePosition = 0.0;
    public double cancoderVelocity = 0.0;
    public double cancoderSupplyVoltage = 0.0;
    public double cancoderPositionRotations = 0.0;
  }

  /**
   * Updates the inputs object with the latest data from hardware
   *
   * @param inputs Inputs to update
   */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Updates that are be called in climb periodic */
  public default void periodicUpdates() {}

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
