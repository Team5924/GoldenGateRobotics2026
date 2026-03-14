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
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
import org.team5924.frc2026.Constants.GeneralTurret;
import org.team5924.frc2026.Constants.TurretLeft;
import org.team5924.frc2026.Constants.TurretRight;
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

  /* Gains Left */
  private final LoggedTunableNumber kPLeft = new LoggedTunableNumber("Turret/Left/kP", 25.0);
  private final LoggedTunableNumber kILeft = new LoggedTunableNumber("Turret/Left/kI", 0.0);
  private final LoggedTunableNumber kDLeft = new LoggedTunableNumber("Turret/Left/kD", 1.00);
  private final LoggedTunableNumber kSLeft = new LoggedTunableNumber("Turret/Left/kS", 0.0);
  private final LoggedTunableNumber kVLeft = new LoggedTunableNumber("Turret/Left/kV", 0.0);
  private final LoggedTunableNumber kALeft = new LoggedTunableNumber("Turret/Left/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocityLeft =
      new LoggedTunableNumber("Turret/Left/MotionCruiseVelocity", 10.0);
  private final LoggedTunableNumber motionAccelerationLeft =
      new LoggedTunableNumber("Turret/Left/MotionAcceleration", 20.0);
  private final LoggedTunableNumber motionJerkLeft =
      new LoggedTunableNumber("Turret/Left/MotionJerk", 0.0);

  /* Gains Right */
  private final LoggedTunableNumber kPRight = new LoggedTunableNumber("Turret/Right/kP", 25.0);
  private final LoggedTunableNumber kIRight = new LoggedTunableNumber("Turret/Right/kI", 0.0);
  private final LoggedTunableNumber kDRight = new LoggedTunableNumber("Turret/Right/kD", 1.00);
  private final LoggedTunableNumber kSRight = new LoggedTunableNumber("Turret/Right/kS", 0.0);
  private final LoggedTunableNumber kVRight = new LoggedTunableNumber("Turret/Right/kV", 0.0);
  private final LoggedTunableNumber kARight = new LoggedTunableNumber("Turret/Right/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocityRight =
      new LoggedTunableNumber("Turret/Right/MotionCruiseVelocity", 10.0);
  private final LoggedTunableNumber motionAccelerationRight =
      new LoggedTunableNumber("Turret/Right/MotionAcceleration", 100.0);
  private final LoggedTunableNumber motionJerkRight =
      new LoggedTunableNumber("Turret/Right/MotionJerk", 0.0);

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

  private final TorqueCurrentFOC currentOut;
  private final PositionVoltage positionOut;
  private final MotionMagicTorqueCurrentFOC motionMagicCurrent;

  private final double minPositionRads;
  private final double maxPositionRads;

  private final Runnable periodicUpdateSlot0;
  private final Runnable periodicUpdateMotionMagic;

  private final boolean isLeft;
  private final String side;

  public TurretIOTalonFX(boolean isLeft) {
    this.isLeft = isLeft;
    side = isLeft ? "Left" : "Right";
    minPositionRads = isLeft ? TurretLeft.MIN_POSITION_RADS : TurretRight.MIN_POSITION_RADS;
    maxPositionRads = isLeft ? TurretLeft.MAX_POSITION_RADS : TurretRight.MAX_POSITION_RADS;

    turretTalon =
        new TalonFX(isLeft ? TurretLeft.CAN_ID : TurretRight.CAN_ID, new CANBus(GeneralTurret.BUS));
    turretCANCoder =
        new CANcoder(
            isLeft ? TurretLeft.CANCODER_ID : TurretRight.CANCODER_ID,
            new CANBus(GeneralTurret.BUS));

    turretTalonConfig = turretTalon.getConfigurator();

    slot0Configs = new Slot0Configs();
    updateSlot0Configs();

    motionMagicConfigs = new MotionMagicConfigs();
    updateMotionMagicConfigs();

    periodicUpdateSlot0 =
        () -> {
          updateSlot0Configs();

          StatusCode statusCode = turretTalonConfig.apply(slot0Configs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("Turret/" + side + "/UpdateSlot0Report", statusCode);
          }
        };

    periodicUpdateMotionMagic =
        () -> {
          updateMotionMagicConfigs();

          StatusCode statusCode = turretTalonConfig.apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("Turret/" + side + "/UpdateStatusCodeReport", statusCode);
          }
        };

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[8];

    statusArray[0] = turretTalonConfig.apply(GeneralTurret.GENERAL_CONFIG);
    statusArray[1] = turretTalonConfig.apply(Constants.GENERIC_OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[2] = turretTalonConfig.apply(Constants.GENERIC_CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[3] =
        turretTalonConfig.apply(
            isLeft ? TurretLeft.SOFTWARE_LIMIT_CONFIGS : TurretRight.SOFTWARE_LIMIT_CONFIGS);
    statusArray[4] =
        turretTalonConfig.apply(
            isLeft ? TurretLeft.FEEDBACK_CONFIGS : TurretRight.FEEDBACK_CONFIGS);
    statusArray[5] = turretTalonConfig.apply(motionMagicConfigs);
    statusArray[6] = turretTalonConfig.apply(slot0Configs);
    statusArray[7] =
        turretCANCoder
            .getConfigurator()
            .apply(isLeft ? TurretLeft.CANCODER_CONFIGS : TurretRight.CANCODER_CONFIGS);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              side + " Turret Configs",
              "Error in applying " + side + " Turret configs!"));

    Logger.recordOutput("Turret/" + side + "/InitConfReport", statusArray);

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

    currentOut = new TorqueCurrentFOC(0.0);
    positionOut = new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true).withSlot(0);
    motionMagicCurrent = new MotionMagicTorqueCurrentFOC(0.0).withSlot(0);

    BaseStatusSignal.waitForAll(0.5, turretPosition, cancoderAbsolutePosition);

    turretTalon.setPosition(0.0);
    turretCANCoder.setPosition(0.0);
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
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(turretPosition, turretVelocity);
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

    inputs.turretPositionCancoder = inputs.cancoderPositionRotations;
  }

  @Override
  public void periodicUpdates() {
    updateLoggedTunableNumbers();
  }

  private void updateSlot0Configs() {
    if (isLeft) {
      slot0Configs.kP = kPLeft.get();
      slot0Configs.kI = kILeft.get();
      slot0Configs.kD = kDLeft.get();
      slot0Configs.kS = kSLeft.get();
      slot0Configs.kV = kVLeft.get();
      slot0Configs.kA = kALeft.get();
    } else {
      slot0Configs.kP = kPRight.get();
      slot0Configs.kI = kIRight.get();
      slot0Configs.kD = kDRight.get();
      slot0Configs.kS = kSRight.get();
      slot0Configs.kV = kVRight.get();
      slot0Configs.kA = kARight.get();
    }
  }

  private void updateMotionMagicConfigs() {
    if (isLeft) {
      motionMagicConfigs.MotionMagicAcceleration = motionAccelerationLeft.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocityLeft.get();
      motionMagicConfigs.MotionMagicJerk = motionJerkLeft.get();
    } else {
      motionMagicConfigs.MotionMagicAcceleration = motionAccelerationRight.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocityRight.get();
      motionMagicConfigs.MotionMagicJerk = motionJerkRight.get();
    }
  }

  private void updateLoggedTunableNumbers() {
    if (isLeft) {
      LoggedTunableNumber.ifChanged(
          hashCode(), periodicUpdateSlot0, kPLeft, kILeft, kDLeft, kSLeft, kVLeft, kALeft);
      LoggedTunableNumber.ifChanged(
          hashCode() + 1,
          periodicUpdateMotionMagic,
          motionAccelerationLeft,
          motionCruiseVelocityLeft,
          motionJerkLeft);
    } else {
      LoggedTunableNumber.ifChanged(
          hashCode(), periodicUpdateSlot0, kPRight, kIRight, kDRight, kSRight, kVRight, kARight);
      LoggedTunableNumber.ifChanged(
          hashCode() + 1,
          periodicUpdateMotionMagic,
          motionAccelerationRight,
          motionCruiseVelocityRight,
          motionJerkRight);
    }
  }

  @Override
  public void runVolts(double volts) {
    turretTalon.setControl(currentOut.withOutput(40 * volts));
  }

  @Override
  public void setPosition(double rads) {
    if (!DriverStation.isEnabled()) {
      stop();
      return;
    }

    setpointRads = clampRads(rads);
    turretTalon.setControl(motionMagicCurrent.withPosition(radsToMotorPosition(setpointRads)));
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
    return MathUtil.clamp(rads, minPositionRads, maxPositionRads);
  }

  private double radsToMotorPosition(double rads) {
    while (rads < minPositionRads) rads += 2 * Math.PI;
    while (rads > maxPositionRads) rads -= 2 * Math.PI;

    if (rads < minPositionRads) turretPosition.getValueAsDouble();
    return Units.radiansToRotations(rads);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition);
  }
}
