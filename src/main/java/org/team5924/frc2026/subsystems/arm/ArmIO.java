package org.team5924.frc2026.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIoInputs {
        public boolean ArmMotorConnected = true;
        public double examplePositiveRads = 0.0;
        public double ArmVelocityRadsPerSec = 0.0;
        public double ArmAppliedVoltage = 0.0;
        public double ArmSupplyCurrentAmps = 0.0;
        public double ArmTorqueCurrentAmps = 0.0;
        public double ArmTempCelcius = 0.0;
        public double ArmPositionRads;
    }

    public default void updateInputs(ArmIoInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void setPosition(double rads) {}

    default void stop() {}
}
