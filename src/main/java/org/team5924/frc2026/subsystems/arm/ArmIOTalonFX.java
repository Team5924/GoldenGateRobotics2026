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

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX armTalon;
  private final StatusSignal<Angle> armPosition;
  private final StatusSignal<AngularVelocity> armVelocity;
  private final StatusSignal<Voltage> armAppliedVoltage;
  private final StatusSignal<Current> armSupplyCurrent;
  private final StatusSignal<Current> armTorqueCurrent;
  private final StatusSignal<Temperature> armTempCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ArmIOTalonFX() {
    armTalon = new TalonFX(Constants.Arm.ARM_CAN_ID, Constants.Arm.BUS);
    armTalon.getConfigurator().apply(Constants.Arm.CONFIG);

    armPosition = armTalon.getPosition();
    armVelocity = armTalon.getVelocity();
    armAppliedVoltage = armTalon.getMotorVoltage();
    armSupplyCurrent = armTalon.getSupplyCurrent();
    armTorqueCurrent = armTalon.getTorqueCurrent();
    armTempCelsius = armTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armPosition,
        armVelocity,
        armAppliedVoltage,
        armSupplyCurrent,
        armTorqueCurrent,
        armTempCelsius);

    armTalon.setPosition(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armMotorConnected =
        BaseStatusSignal.refreshAll(
                armPosition,
                armVelocity,
                armAppliedVoltage,
                armSupplyCurrent,
                armTorqueCurrent,
                armTempCelsius)
            .isOK();
    inputs.armPositionRads =
        Units.rotationsToRadians(armPosition.getValueAsDouble()) / Constants.Arm.ARM_REDUCTION;
    inputs.armVelocityRadsPerSec =
        Units.rotationsToRadians(armVelocity.getValueAsDouble()) / Constants.Arm.ARM_REDUCTION;
    inputs.armAppliedVoltage = armAppliedVoltage.getValueAsDouble();
    inputs.armSupplyCurrentAmps = armSupplyCurrent.getValueAsDouble();
    inputs.armTorqueCurrentAmps = armTorqueCurrent.getValueAsDouble();
    inputs.armTempCelsius = armTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    armTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    armTalon.setControl(
        positionOut.withPosition(Units.radiansToRotations(rads) * Constants.ShooterHood.REDUCTION));
  }

  @Override
  public void stop() {
    armTalon.stopMotor();
  }
}
