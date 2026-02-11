/*
 * ArmIO.java
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

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean armMotorConnected = true;
    public double armPositionRads = 0.0;
    public double armVelocityRadsPerSec = 0.0;
    public double armAppliedVoltage = 0.0;
    public double armSupplyCurrentAmps = 0.0;
    public double armTorqueCurrentAmps = 0.0;
    public double armTempCelsius = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void setPosition(double rads) {}

  default void stop() {}
}
