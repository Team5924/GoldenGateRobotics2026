/*
 * TurretIOTalonFX.java
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

package org.team5924.frc2026.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX turretTalon;
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Voltage> turretAppliedVoltage;
  private final StatusSignal<Current> turretSupplyCurrent;
  private final StatusSignal<Current> turretTorqueCurrent;
  private final StatusSignal<Temperature> turretTempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public turretSystemIOTalonFX() {
    turretTalon = new TalonFX(Constants.Turret.CAN_ID, new CANBus(Constants.Turret.BUS));
    turretTalon.getConfigurator().apply(Constants.Turret.CONFIG);

    // Get select status signals and set update frequency
    turretPosition = turretTalon.getPosition();
    turretVelocity = turretTalon.getVelocity();
    turretAppliedVoltage = turretTalon.getMotorVoltage();
    turretSupplyCurrent = turretTalon.getSupplyCurrent();
    turretTorqueCurrent = turretTalon.getTorqueCurrent();
    turretTempCelsius = turretTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        turretPosition,
        turretVelocity,
        turretAppliedVoltage,
        turretSupplyCurrent,
        turretTorqueCurrent,
        turretTempCelsius);

    turretTalon.setPosition(0);
  }

  @Override
  public void updateInputs(turretIOInputs inputs) {
    inputs.turretMotorConnected =
        BaseStatusSignal.refreshAll(
                turretPosition,
                turretVelocity,
                turretAppliedVoltage,
                turretSupplyCurrent,
                turretTorqueCurrent,
                turretTempCelsius)
            .isOK();
    inputs.turretPositionRads =
        Units.rotationsToRadians(turretPosition.getValueAsDouble())
            / Constants.Turret.TURRET_REDUCTION;
    inputs.turretVelocityRadsPerSec =
        Units.rotationsToRadians(turretVelocity.getValueAsDouble())
            / Constants.Turret.TURRET_REDUCTION;
    inputs.turretAppliedVoltage = turretAppliedVoltage.getValueAsDouble();
    inputs.turretSupplyCurrentAmps = turretSupplyCurrent.getValueAsDouble();
    inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
    inputs.turretTempCelsius = turretTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    turretTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    turretTalon.stopMotor();
  }
}
