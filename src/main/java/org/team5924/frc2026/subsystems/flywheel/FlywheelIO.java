/*
 * FlywheelIO.java
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

package org.team5924.frc2026.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean motorConnected = true;
    public double position = 0.0;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public double followerSupplyCurrentAmps;
    public double followerTempCelsius;

    public double setpointVelocity;

    public double motionMagicVelocityTarget = 0.0;
    public double acceleration = 0.0;
  }

  /**
   * Updates the inputs object with the latest data from hardware
   *
   * @param inputs Inputs to update
   */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Updates that are be called in flywheel periodic */
  public default void periodicUpdates() {}

  /**
   * Sets the flywheel motor to the specified voltage
   *
   * @param volts number of volts
   */
  public default void runVolts(double volts) {}

  /**
   * Runs the shooter roller motor at the specified velocity
   *
   * @param velocity velocity in rads/sec
   */
  public default void setVelocity(double velocity) {}

  /** stops the motor */
  default void stop() {}
}
