/*
 * TurretIO.java
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

package org.team5924.frc2026.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean turretMotorConnected = true;
    public double turretPositionRads = 0.0;
    public double turretVelocityRadsPerSec = 0.0;
    public double turretAppliedVoltage = 0.0;
    public double turretSupplyCurrentAmps = 0.0;
    public double turretTorqueCurrentAmps = 0.0;
    public double turretTempCelsius = 0.0;

    public double motionMagicVelocityTarget = 0.0;
    public double motionMagicPositionTarget = 0.0;

    public double setpointMeters = 0.0;
    public double acceleration = 0.0;

    public boolean cancoderConnected = true;
    public double cancoderPosition = 0.0;
    public double cancoderVelocity = 0.0;
    public double cancoderSupplyVoltage = 0.0;
  }

  /**
   * Updates the inputs object with the latest data from hardware
   *
   * @param inputs Inputs to update
   */
  public default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Sets the subsystem motor to the specified voltage
   *
   * @param volts number of volts
   */
  public default void runVolts(double volts) {}

  /**
   * Sets the subsystem motor to a specified angle
   *
   * @param rads target angle
   */
  public default void setPosition(double rads) {}

  /** stops the motor */
  default void stop() {}

  /** sdfsdf */
  default void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {}
}
