/*
 * IntakePivotIO.java
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

package org.team5924.frc2026.subsystems.pivots.intakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public boolean intakePivotMotorConnected = true;
    public double intakePivotPosition = 0.0;
    public double intakePivotPositionRads = 0.0;
    public double intakePivotPositionCancoder = 0.0;
    public double intakePivotVelocityRadsPerSec = 0.0;
    public double intakePivotAppliedVoltage = 0.0;
    public double intakePivotSupplyCurrentAmps = 0.0;
    public double intakePivotTorqueCurrentAmps = 0.0;
    public double intakePivotTempCelsius = 0.0;

    public double motionMagicVelocityTarget = 0.0;
    public double motionMagicPositionTarget = 0.0;

    public double setpointRads = 0.0;
    public double acceleration = 0.0;

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
  public default void updateInputs(IntakePivotIOInputs inputs) {}

  /** Updates that are be called in intakePivot periodic */
  public default void periodicUpdates() {}

  /**
   * Sets the intakePivot motor to the specified voltage
   *
   * @param volts number of volts
   */
  public default void runVolts(double volts) {}

  /**
   * Sets the intakePivot motor to a specified angle
   *
   * @param rads target angle
   */
  public default void setPosition(double rads) {}

  /** Holds the intakePivot motor at a set position */
  public default void holdPosition(double rads) {}

  /** stops the motor */
  default void stop() {}

  /** Sets the intakePivot position to specified rads from center */
  default void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {}
}
