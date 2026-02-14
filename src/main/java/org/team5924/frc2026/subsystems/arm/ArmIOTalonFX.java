/*
 * ArmIOTalonFX.java
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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
import edu.wpi.first.wpilibj.DigitalInput;
import org.team5924.frc2026.Constants;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX ArmTalon;
  private final StatusSignal<Angle> ArmPosition;
  private final StatusSignal<AngularVelocity> ArmVelocity;
  private final StatusSignal<Voltage> ArmAppliedVoltage;
  private final StatusSignal<Current> ArmSupplyCurrent;
  private final StatusSignal<Current> ArmTorqueCurrent;
  private final StatusSignal<Temperature> ArmTempCelsius;

  private final DigitalInput beamBreak = new DigitalInput(0);

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ArmIOTalonFX() {
    ArmTalon = new TalonFX(Constants.Arm.CAN_ID, new CANBus(Constants.Arm.BUS));
    ArmTalon.getConfigurator().apply(Constants.Arm.CONFIG);

    // Get select status signals and set update frequency
    ArmPosition = ArmTalon.getPosition();
    ArmVelocity = ArmTalon.getVelocity();
    ArmAppliedVoltage = ArmTalon.getMotorVoltage();
    ArmSupplyCurrent = ArmTalon.getSupplyCurrent();
    ArmTorqueCurrent = ArmTalon.getTorqueCurrent();
    ArmTempCelsius = ArmTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        ArmPosition,
        ArmVelocity,
        ArmAppliedVoltage,
        ArmSupplyCurrent,
        ArmTorqueCurrent,
        ArmTempCelsius);

    ArmTalon.setPosition(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.ArmMotorConnected =
        BaseStatusSignal.refreshAll(
                ArmPosition,
                ArmVelocity,
                ArmAppliedVoltage,
                ArmSupplyCurrent,
                ArmTorqueCurrent,
                ArmTempCelsius)
            .isOK();
    inputs.ArmPositionRads =
        Units.rotationsToRadians(ArmPosition.getValueAsDouble()) / Constants.Example.REDUCTION;
    inputs.ArmVelocityRadsPerSec =
        Units.rotationsToRadians(ArmVelocity.getValueAsDouble()) / Constants.Example.REDUCTION;
    inputs.ArmAppliedVoltage = ArmAppliedVoltage.getValueAsDouble();
    inputs.ArmSupplyCurrentAmps = ArmSupplyCurrent.getValueAsDouble();
    inputs.ArmTorqueCurrentAmps = ArmTorqueCurrent.getValueAsDouble();
    inputs.ArmTempCelsius = ArmTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    ArmTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    ArmTalon.stopMotor();
  }
}
