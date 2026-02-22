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
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class TurretIOTalonFX implements TurretIO {
  /* Hardware */
  private final TalonFX turretTalon;
  private final CANcoder turretCANCoder;

  /* Configurators */
  private TalonFXConfigurator turretTalonConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointRads;

  /* Gains */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 3.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.07);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.13);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.4);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Turret/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Turret/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerk = new LoggedTunableNumber("Turret/MotionJerk", 0.0);

  /* Status Signals */
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Voltage> turretAppliedVoltage;
  private final StatusSignal<Current> turretSupplyCurrent;
  private final StatusSignal<Current> turretTorqueCurrent;
  private final StatusSignal<Temperature> turretTempCelsius;

  private final StatusSignal<Angle> cancoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private final StatusSignal<Voltage> cancoderSupplyVoltage;
  private final StatusSignal<Angle> cancoderPositionRotations;

  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageOut;
  private final PositionVoltage positionOut;
  private final MotionMagicVoltage magicMotionVoltage;

  public TurretIOTalonFX() {
    turretTalon = new TalonFX(Constants.Turret.CAN_ID, new CANBus(Constants.Turret.BUS));
    turretCANCoder = new CANcoder(Constants.Turret.CANCODER_ID);

    turretTalonConfig = turretTalon.getConfigurator();

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[7];

    statusArray[0] = turretTalonConfig.apply(Constants.Turret.CONFIG);
    statusArray[1] = turretTalonConfig.apply(slot0Configs);
    statusArray[2] = turretTalonConfig.apply(motionMagicConfigs);
    statusArray[3] = turretTalonConfig.apply(Constants.Turret.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[4] = turretTalonConfig.apply(Constants.Turret.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[5] = turretTalonConfig.apply(Constants.Turret.FEEDBACK_CONFIGS);
    statusArray[6] = turretCANCoder.getConfigurator().apply(Constants.Turret.CANCODER_CONFIG);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING, "Turret Configs", "Error in turret configs!"));

    Logger.recordOutput("Turret/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    turretPosition = turretTalon.getPosition();
    turretVelocity = turretTalon.getVelocity();
    turretAppliedVoltage = turretTalon.getMotorVoltage();
    turretSupplyCurrent = turretTalon.getSupplyCurrent();
    turretTorqueCurrent = turretTalon.getTorqueCurrent();
    turretTempCelsius = turretTalon.getDeviceTemp();

    cancoderAbsolutePosition = turretCANCoder.getAbsolutePosition();
    cancoderVelocity = turretCANCoder.getVelocity();
    cancoderSupplyVoltage = turretCANCoder.getSupplyVoltage();
    cancoderPositionRotations = turretCANCoder.getPosition();

    closedLoopReferenceSlope = turretTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        turretPosition,
        turretVelocity,
        turretAppliedVoltage,
        turretSupplyCurrent,
        turretTorqueCurrent,
        turretTempCelsius,
        cancoderAbsolutePosition,
        cancoderVelocity,
        cancoderSupplyVoltage,
        cancoderPositionRotations,
        closedLoopReferenceSlope);

    voltageOut = new VoltageOut(0.0);
    positionOut = new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true).withSlot(0);
    magicMotionVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true).withSlot(0);

    BaseStatusSignal.waitForAll(0.5, cancoderAbsolutePosition);
    turretCANCoder.setPosition(0.0);
    turretTalon.setPosition(0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretMotorConnected =
        BaseStatusSignal.refreshAll(
                turretPosition,
                turretVelocity,
                turretAppliedVoltage,
                turretSupplyCurrent,
                turretTorqueCurrent,
                turretTempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.cancoderConnected =
        BaseStatusSignal.refreshAll(
                cancoderAbsolutePosition,
                cancoderVelocity,
                cancoderSupplyVoltage,
                cancoderPositionRotations)
            .isOK();

    inputs.turretPosition =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(turretPosition, turretVelocity)
            / Constants.Turret.MOTOR_TO_MECHANISM;
    inputs.turretPositionRads = Units.rotationsToRadians(inputs.turretPosition);

    inputs.turretVelocityRadsPerSec = Units.rotationsToRadians(turretVelocity.getValueAsDouble());
    inputs.turretAppliedVoltage = turretAppliedVoltage.getValueAsDouble();
    inputs.turretSupplyCurrentAmps = turretSupplyCurrent.getValueAsDouble();
    inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
    inputs.turretTempCelsius = turretTempCelsius.getValueAsDouble();

    inputs.motionMagicVelocityTarget =
        motorPositionToRads(turretTalon.getClosedLoopReferenceSlope().getValueAsDouble());
    inputs.motionMagicPositionTarget =
        motorPositionToRads(turretTalon.getClosedLoopReference().getValueAsDouble());

    inputs.setpointRads = setpointRads;

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;

    inputs.cancoderAbsolutePosition = cancoderAbsolutePosition.getValueAsDouble();
    inputs.cancoderVelocity = cancoderVelocity.getValueAsDouble();
    inputs.cancoderSupplyVoltage = cancoderSupplyVoltage.getValueAsDouble();
    inputs.cancoderPositionRotations = cancoderPositionRotations.getValueAsDouble();

    inputs.turretPositionCancoder =
        inputs.cancoderPositionRotations / Constants.Turret.CANCODER_TO_MECHANISM;
  }

  @Override
  public void periodicUpdates() {
    updateLoggedTunableNumbers();
  }

  private void updateLoggedTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        0,
        () -> {
          slot0Configs.kP = kP.get();
          slot0Configs.kI = kI.get();
          slot0Configs.kD = kD.get();
          slot0Configs.kS = kS.get();
          slot0Configs.kV = kV.get();
          slot0Configs.kA = kA.get();

          StatusCode statusCode = turretTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Turret Slot 0 Configs",
                    "Error in periodically updating turret Slot0 configs!"));

            Logger.recordOutput("Turret/UpdateSlot0Report", statusCode);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);

    LoggedTunableNumber.ifChanged(
        0,
        () -> {
          motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
          motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
          motionMagicConfigs.MotionMagicJerk = motionJerk.get();

          StatusCode statusCode = turretTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Turret Motion Magic Configs",
                    "Error in periodically updating turret MotionMagic configs!"));

            Logger.recordOutput("Turret/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void runVolts(double volts) {
    turretTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    if (!DriverStation.isEnabled()) {
      stop();
      return;
    }

    setpointRads = clampRads(rads);
    turretTalon.setControl(magicMotionVoltage.withPosition(radsToMotorPosition(setpointRads)));
  }

  @Override
  public void holdPosition(double rads) {
    turretTalon.setControl(positionOut.withPosition(radsToMotorPosition(rads)));
  }

  @Override
  public void stop() {
    turretTalon.stopMotor();
  }

  private double clampRads(double rads) {
    return MathUtil.clamp(
        rads, Constants.Turret.MIN_POSITION_RADS, Constants.Turret.MAX_POSITION_RADS);
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads) * Constants.Turret.MOTOR_TO_MECHANISM;
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition) / Constants.Turret.MOTOR_TO_MECHANISM;
  }
}
