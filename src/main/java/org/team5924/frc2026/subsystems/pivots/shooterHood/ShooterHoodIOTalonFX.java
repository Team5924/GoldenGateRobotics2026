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
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ShooterHoodIOTalonFX implements ShooterHoodIO {
  private final TalonFX shooterHoodTalon;
  private final StatusSignal<Angle> shooterHoodPosition;
  private final StatusSignal<AngularVelocity> shooterHoodVelocity;
  private final StatusSignal<Voltage> shooterHoodAppliedVoltage;
  private final StatusSignal<Current> shooterHoodSupplyCurrent;
  private final StatusSignal<Current> shooterHoodTorqueCurrent;
  private final StatusSignal<Temperature> shooterHoodTempCelsius;

  private TalonFXConfigurator shooterHoodTalonConfig;

  private final CANcoder cancoder;
  private final StatusSignal<Angle> cancoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private final StatusSignal<Voltage> cancoderSupplyVoltage;
  private final StatusSignal<Angle> cancoderPositionRotations;

  private final Slot0Configs slot0Configs;
  private double setpoint;

  private final LoggedTunableNumber kP = new LoggedTunableNumber("ShooterHood/kP", 1.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("ShooterHood/kI", 0.0);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("ShooterHood/kD", 0.00); // TODO: Change values later
  private final LoggedTunableNumber kS = new LoggedTunableNumber("ShooterHood/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("ShooterHood/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("ShooterHood/kA", 0.00);

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  public ShooterHoodIOTalonFX() {
    shooterHoodTalon = new TalonFX(Constants.ShooterHood.CAN_ID, Constants.ShooterHood.BUS);
    shooterHoodTalonConfig = shooterHoodTalon.getConfigurator();

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();

    cancoder = new CANcoder(Constants.ShooterHood.CANCODER_ID);
    cancoder
        .getConfigurator()
        .apply(new CANcoderConfiguration().withMagnetSensor(Constants.ShooterHood.CANCODER_CONFIG));

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[7];

    statusArray[0] = shooterHoodTalonConfig.apply(Constants.ShooterHood.CONFIG);
    statusArray[1] = shooterHoodTalonConfig.apply(slot0Configs);
    statusArray[2] = shooterHoodTalonConfig.apply(Constants.ShooterHood.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[3] = shooterHoodTalonConfig.apply(Constants.ShooterHood.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[4] = shooterHoodTalonConfig.apply(Constants.ShooterHood.FEEDBACK_CONFIGS);
    statusArray[5] = cancoder.getConfigurator().apply(Constants.ShooterHood.CANCODER_CONFIG);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(NotificationLevel.WARNING, "Turret: Error in configs", ""));

    Logger.recordOutput("Turret/InitConfReport", statusArray);

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

    inputs.shooterHoodPosition =
        shooterHoodPosition.getValueAsDouble()
            / Constants.ShooterHood.MOTOR_TO_MECHANISM
            / Constants.ShooterHood.MOTOR_TO_CANCODER;
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
    inputs.setpointRads = setpoint;

    inputs.cancoderAbsolutePosition = cancoderAbsolutePosition.getValueAsDouble();
    inputs.cancoderVelocity = cancoderVelocity.getValueAsDouble();
    inputs.cancoderSupplyVoltage = cancoderSupplyVoltage.getValueAsDouble();
    inputs.cancoderPositionRotations = cancoderPositionRotations.getValueAsDouble();

    inputs.shooterHoodPositionCancoder =
        (inputs.cancoderPositionRotations) / Constants.ShooterHood.CANCODER_TO_MECHANISM;

    updatedLoggedTunableNumbers();
  }

  private void updatedLoggedTunableNumbers() { // TODO: check if updated
    LoggedTunableNumber.ifChanged(
        0,
        () -> {
          slot0Configs.kP = kP.get();
          slot0Configs.kI = kI.get();
          slot0Configs.kD = kD.get();
          slot0Configs.kS = kS.get();
          slot0Configs.kV = kV.get();
          slot0Configs.kA = kA.get();

          shooterHoodTalon.getConfigurator().apply(slot0Configs);
        },
        kP,
        kI,
        kD,
        kS,
        kV);
  }

  @Override
  public void runVolts(double volts) {
    shooterHoodTalon.setControl(voltageOut.withOutput(volts));
  }

  // @Override
  // public void setPosition(double rads) {
  //   shooterHoodTalon.setControl(
  //       positionOut.withPosition(Units.radiansToRotations(rads) *
  // Constants.ShooterHood.CANCODER_TO_MECHANISM));
  // }

  @Override
  public void stop() {
    shooterHoodTalon.stopMotor();
  }
}
