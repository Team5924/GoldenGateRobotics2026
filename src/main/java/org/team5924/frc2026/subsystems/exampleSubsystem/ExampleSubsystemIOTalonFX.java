/*
 * ExampleSubsystemIOTalonFX.java
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.team5924.frc2026.Constants;

public class ExampleSubsystemIOTalonFX implements ExampleSubsystemIO {
    private final TalonFX exampleSubsystemTalon;
    private final StatusSignal<Angle> exampleSubsystemPosition;
    private final StatusSignal<AngularVelocity> exampleSubsystemVelocity;
    private final StatusSignal<Voltage> exampleSubsystemAppliedVoltage;
    private final StatusSignal<Current> exampleSubsystemSupplyCurrent;
    private final StatusSignal<Current> exampleSubsystemTorqueCurrent;
    private final StatusSignal<Temperature> exampleSubsystemTempCelsius;

    // Single shot for voltage mode, robot loop will call continuously
    private final VoltageOut voltageOut =
            new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
    private final PositionVoltage positionOut =
            new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

    public ExampleSubsystemIOTalonFX() {
        exampleSubsystemTalon =
                new TalonFX(Constants.ExampleSubsystem.CAN_ID, Constants.ExampleSubsystem.BUS);
        exampleSubsystemTalon.getConfigurator().apply(Constants.ExampleSubsystem.CONFIG);

        // Get select status signals and set update frequency
        exampleSubsystemPosition = exampleSubsystemTalon.getPosition();
        exampleSubsystemVelocity = exampleSubsystemTalon.getVelocity();
        exampleSubsystemAppliedVoltage = exampleSubsystemTalon.getMotorVoltage();
        exampleSubsystemSupplyCurrent = exampleSubsystemTalon.getSupplyCurrent();
        exampleSubsystemTorqueCurrent = exampleSubsystemTalon.getTorqueCurrent();
        exampleSubsystemTempCelsius = exampleSubsystemTalon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                exampleSubsystemPosition,
                exampleSubsystemVelocity,
                exampleSubsystemAppliedVoltage,
                exampleSubsystemSupplyCurrent,
                exampleSubsystemTorqueCurrent,
                exampleSubsystemTempCelsius);

        exampleSubsystemTalon.setPosition(0);
    }

    @Override
    public void updateInputs(ExampleSubsystemIOInputs inputs) {
        inputs.exampleSubsystemMotorConnected =
                BaseStatusSignal.refreshAll(
                                exampleSubsystemPosition,
                                exampleSubsystemVelocity,
                                exampleSubsystemAppliedVoltage,
                                exampleSubsystemSupplyCurrent,
                                exampleSubsystemTorqueCurrent,
                                exampleSubsystemTempCelsius)
                        .isOK();
        inputs.exampleSubsystemPositionRads =
                Units.rotationsToRadians(exampleSubsystemPosition.getValueAsDouble())
                        / Constants.ExampleSubsystem.REDUCTION;
        inputs.exampleSubsystemVelocityRadsPerSec =
                Units.rotationsToRadians(exampleSubsystemVelocity.getValueAsDouble())
                        / Constants.ExampleSubsystem.REDUCTION;
        inputs.exampleSubsystemAppliedVolts = exampleSubsystemAppliedVoltage.getValueAsDouble();
        inputs.exampleSubsystemSupplyCurrentAmps = exampleSubsystemSupplyCurrent.getValueAsDouble();
        inputs.exampleSubsystemTorqueCurrentAmps = exampleSubsystemTorqueCurrent.getValueAsDouble();
        inputs.exampleSubsystemTempCelsius = exampleSubsystemTempCelsius.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        exampleSubsystemTalon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stop() {
        exampleSubsystemTalon.stopMotor();
    }
}
