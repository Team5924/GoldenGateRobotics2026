/*
 * ShooterFlywheelIO.java
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

package org.team5924.frc2026.subsystems.rollers.shooterFlywheel;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO {
  @AutoLog
  public static class ShooterFlywheelIOInputs {
    public boolean shooterFlywheelMotorConnected = true;
    public double shooterFlywheelPosition = 0.0;
    public double shooterFlywheelPositionRads = 0.0;
    public double shooterFlywheelVelocityRadsPerSec = 0.0;
    public double shooterFlywheelAppliedVoltage = 0.0;
    public double shooterFlywheelSupplyCurrentAmps = 0.0;
    public double shooterFlywheelTorqueCurrentAmps = 0.0;
    public double shooterFlywheelTempCelsius = 0.0;

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
  public default void updateInputs(ShooterFlywheelIOInputs inputs) {}

  /** Updates that are be called in shooterFlywheel periodic */
  public default void periodicUpdates() {}

  /**
   * Sets the shooterFlywheel motor to the specified voltage
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
