/*
 * ExampleSubsystemIO.java
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

package org.team5924.frc2026.subsystems.exampleSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ExampleSubsystemIO {
    @AutoLog
    public static class ExampleSubsystemIOInputs {
        public boolean exampleSubsystemMotorConnected = true;
        public double exampleSubsystemPositionRads = 0.0;
        public double exampleSubsystemVelocityRadsPerSec = 0.0;
        public double exampleSubsystemAppliedVolts = 0.0;
        public double exampleSubsystemSupplyCurrentAmps = 0.0;
        public double exampleSubsystemTorqueCurrentAmps = 0.0;
        public double exampleSubsystemTempCelsius = 0.0;
    }

    /**
     * Updates the inputs object with the latest data from hardware
     *
     * @param inputs Inputs to update
     */
    public default void updateInputs(ExampleSubsystemIOInputs inputs) {}

    /**
     * Sets the subsystem motor to the specified voltage
     *
     * @param volts number of volts
     */
    public default void setVoltage(double volts) {}

    public default void setPosition(double rads) {}

    /** stops the motor */
    default void stop() {}
}
