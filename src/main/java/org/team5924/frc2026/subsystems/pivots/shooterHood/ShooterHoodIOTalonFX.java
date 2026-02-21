/*
 * ShooterHoodIOTalonFX.java
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

package org.team5924.frc2026.subsystems.pivots.shooterHood;

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
import org.team5924.frc2026.Constants;

public class ShooterHoodIOTalonFX implements ShooterHoodIO {
  private final TalonFX shooterHoodTalon;
  private final StatusSignal<Angle> shooterHoodPosition;
  private final StatusSignal<AngularVelocity> shooterHoodVelocity;
  private final StatusSignal<Voltage> shooterHoodAppliedVoltage;
  private final StatusSignal<Current> shooterHoodSupplyCurrent;
  private final StatusSignal<Current> shooterHoodTorqueCurrent;
  private final StatusSignal<Temperature> shooterHoodTempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ShooterHoodIOTalonFX() {
    shooterHoodTalon =
        new TalonFX(Constants.ShooterHood.CAN_ID, new CANBus(Constants.ShooterHood.BUS));
    shooterHoodTalon.getConfigurator().apply(Constants.ShooterHood.CONFIG);

    // Get select status signals and set update frequency
    shooterHoodPosition = shooterHoodTalon.getPosition();
    shooterHoodVelocity = shooterHoodTalon.getVelocity();
    shooterHoodAppliedVoltage = shooterHoodTalon.getMotorVoltage();
    shooterHoodSupplyCurrent = shooterHoodTalon.getSupplyCurrent();
    shooterHoodTorqueCurrent = shooterHoodTalon.getTorqueCurrent();
    shooterHoodTempCelsius = shooterHoodTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        shooterHoodPosition,
        shooterHoodVelocity,
        shooterHoodAppliedVoltage,
        shooterHoodSupplyCurrent,
        shooterHoodTorqueCurrent,
        shooterHoodTempCelsius);

    shooterHoodTalon.setPosition(0);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    inputs.shooterHoodMotorConnected =
        BaseStatusSignal.refreshAll(
                shooterHoodPosition,
                shooterHoodVelocity,
                shooterHoodAppliedVoltage,
                shooterHoodSupplyCurrent,
                shooterHoodTorqueCurrent,
                shooterHoodTempCelsius)
            .isOK();
    inputs.shooterHoodPositionRads =
        Units.rotationsToRadians(shooterHoodPosition.getValueAsDouble())
            / Constants.ShooterHood.REDUCTION;
    inputs.shooterHoodVelocityRadsPerSec =
        Units.rotationsToRadians(shooterHoodVelocity.getValueAsDouble())
            / Constants.ShooterHood.REDUCTION;
    inputs.shooterHoodAppliedVoltage = shooterHoodAppliedVoltage.getValueAsDouble();
    inputs.shooterHoodSupplyCurrentAmps = shooterHoodSupplyCurrent.getValueAsDouble();
    inputs.shooterHoodTorqueCurrentAmps = shooterHoodTorqueCurrent.getValueAsDouble();
    inputs.shooterHoodTempCelsius = shooterHoodTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    shooterHoodTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    shooterHoodTalon.setControl(
        positionOut.withPosition(Units.radiansToRotations(rads) * Constants.ShooterHood.REDUCTION));
  }

  @Override
  public void stop() {
    shooterHoodTalon.stopMotor();
  }
}
