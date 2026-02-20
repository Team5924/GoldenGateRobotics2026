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
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;

public class ShooterHoodIOTalonFX implements ShooterHoodIO {
  private final TalonFX shooterHoodTalon;
  private final StatusSignal<Angle> shooterHoodPosition;
  private final StatusSignal<AngularVelocity> shooterHoodVelocity;
  private final StatusSignal<Voltage> shooterHoodAppliedVoltage;
  private final StatusSignal<Current> shooterHoodSupplyCurrent;
  private final StatusSignal<Current> shooterHoodTorqueCurrent;
  private final StatusSignal<Temperature> shooterHoodTempCelsius;

  private final CANcoder cancoder;
  private final StatusSignal<Angle> cancoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private final StatusSignal<Voltage> cancoderSupplyVoltage;
  private final StatusSignal<Angle> cancoderPositionRotations;

  private boolean isCancoderOffset = false;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ShooterHoodIOTalonFX() {
    shooterHoodTalon = new TalonFX(Constants.ShooterHood.CAN_ID, Constants.ShooterHood.BUS);
    shooterHoodTalon.getConfigurator().apply(Constants.ShooterHood.CONFIG);

    cancoder = new CANcoder(Constants.ShooterHood.CANCODER_ID);
    cancoder.getConfigurator().apply(
      new CANcoderConfiguration()
        .withMagnetSensor(Constants.ShooterHood.CANCODER_CONFIG));

    // Get select status signals and set update frequency
    shooterHoodPosition = shooterHoodTalon.getPosition();
    shooterHoodVelocity = shooterHoodTalon.getVelocity();
    shooterHoodAppliedVoltage = shooterHoodTalon.getMotorVoltage();
    shooterHoodSupplyCurrent = shooterHoodTalon.getSupplyCurrent();
    shooterHoodTorqueCurrent = shooterHoodTalon.getTorqueCurrent();
    shooterHoodTempCelsius = shooterHoodTalon.getDeviceTemp();

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    cancoderVelocity = cancoder.getVelocity();
    cancoderSupplyVoltage = cancoder.getSupplyVoltage();
    cancoderPositionRotations = cancoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        shooterHoodPosition,
        shooterHoodVelocity,
        shooterHoodAppliedVoltage,
        shooterHoodSupplyCurrent,
        shooterHoodTorqueCurrent,
        shooterHoodTempCelsius);

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0,
        cancoderAbsolutePosition,
        cancoderVelocity,
        cancoderSupplyVoltage,
        cancoderPositionRotations);


    BaseStatusSignal.waitForAll(0.5, cancoderAbsolutePosition);
    cancoder.setPosition(0.0);
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

    inputs.cancoderConnected =
        BaseStatusSignal.refreshAll(
                cancoderAbsolutePosition,
                cancoderVelocity,
                cancoderSupplyVoltage,
                cancoderPositionRotations)
            .isOK();



    Logger.recordOutput("Turret/isCancoderOffset", isCancoderOffset);

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

    inputs.cancoderAbsolutePosition = cancoderAbsolutePosition.getValueAsDouble();
    inputs.cancoderVelocity = cancoderVelocity.getValueAsDouble();
    inputs.cancoderSupplyVoltage = cancoderSupplyVoltage.getValueAsDouble();
    inputs.cancoderPositionRotations = cancoderPositionRotations.getValueAsDouble();

    inputs.shooterHoodPositionRads =
        (inputs.cancoderPositionRotations) / Constants.ShooterHood.CANCODER_REDUCTION;
    // this was shooterHoodPositionRotations, but I changed it to rads. I don't know if this is
    // right or not.
  }

  @Override
  public void runVolts(double volts) {
    shooterHoodTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    shooterHoodTalon.setControl(
        positionOut.withPosition(Units.radiansToRotations(rads) * Constants.ShooterHood.CANCODER_REDUCTION));
  }

  @Override
  public void stop() {
    shooterHoodTalon.stopMotor();
  }
}
