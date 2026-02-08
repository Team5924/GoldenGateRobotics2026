/*
 * ArmPracticeIO.java
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

package org.team5924.frc2026.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmPracticeIO {
  @AutoLog
  public static class ArmPracticeIOInputs {
    public boolean armPracticeMotorConnected = true;
    public double armPracticePositionRads = 0.0;
    public double armPracticeVelocityRadsPerSec = 0.0;
    public double armPracticeAppliedVoltage = 0.0;
    public double armPracticeSupplyCurrentAmps = 0.0;
    public double armPracticeTorqueCurrentAmps = 0.0;
    public double armPracticeTempCelcius = 0.0;
  }

  /**
   * Updates the inputs object with the latest data from hardware
   *
   * @param inputs Inputs to update
   */
  public default void updateInputs(ArmPracticeIOInputs inputs) {}

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
