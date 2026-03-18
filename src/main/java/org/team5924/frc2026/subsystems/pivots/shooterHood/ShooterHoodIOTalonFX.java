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
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import org.team5924.frc2026.Constants.GeneralShooterHood;
import org.team5924.frc2026.Constants.ShooterHoodLeft;
import org.team5924.frc2026.Constants.ShooterHoodRight;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ShooterHoodIOTalonFX implements ShooterHoodIO {
  /* Hardware */
  private final TalonFX talon;
  private final CANcoder cancoder;

  /* Configurators */
  private TalonFXConfigurator talonConfig;

  /* Configs */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointRads;

  /* Gains Left */
  private final LoggedTunableNumber kPLeft = new LoggedTunableNumber("ShooterHood/Left/kP", 250.0);
  private final LoggedTunableNumber kILeft = new LoggedTunableNumber("ShooterHood/Left/kI", 0.0);
  private final LoggedTunableNumber kDLeft = new LoggedTunableNumber("ShooterHood/Left/kD", 5.0);
  private final LoggedTunableNumber kSLeft = new LoggedTunableNumber("ShooterHood/Left/kS", 0.0);
  private final LoggedTunableNumber kVLeft = new LoggedTunableNumber("ShooterHood/Left/kV", 0.0);
  private final LoggedTunableNumber kALeft = new LoggedTunableNumber("ShooterHood/Left/kA", 0.0);

  private final LoggedTunableNumber motionCruiseVelocityLeft =
      new LoggedTunableNumber("ShooterHood/Left/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAccelerationLeft =
      new LoggedTunableNumber("ShooterHood/Left/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerkLeft =
      new LoggedTunableNumber("ShooterHood/Left/MotionJerk", 0.0);

  /* Gains Right */
  private final LoggedTunableNumber kPRight =
      new LoggedTunableNumber("ShooterHood/Right/kP", 250.0);
  private final LoggedTunableNumber kIRight = new LoggedTunableNumber("ShooterHood/Right/kI", 0.0);
  private final LoggedTunableNumber kDRight = new LoggedTunableNumber("ShooterHood/Right/kD", 5.0);
  private final LoggedTunableNumber kSRight = new LoggedTunableNumber("ShooterHood/Right/kS", 0.0);
  private final LoggedTunableNumber kVRight = new LoggedTunableNumber("ShooterHood/Right/kV", 0.0);
  private final LoggedTunableNumber kARight = new LoggedTunableNumber("ShooterHood/Right/kA", 0.0);

  private final LoggedTunableNumber motionCruiseVelocityRight =
      new LoggedTunableNumber("ShooterHood/Right/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAccelerationRight =
      new LoggedTunableNumber("ShooterHood/Right/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerkRight =
      new LoggedTunableNumber("ShooterHood/Right/MotionJerk", 0.0);

  /* Status Signals */
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final StatusSignal<Angle> cancoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private final StatusSignal<Voltage> cancoderSupplyVoltage;
  private final StatusSignal<Angle> cancoderPositionRotations;

  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageOut;
  private final PositionVoltage positionOut;
  private final MotionMagicTorqueCurrentFOC motionMagicCurrent;

  private final Runnable periodicUpdateSlot0;
  private final Runnable periodicUpdateMotionMagic;

  private final boolean isLeft;
  private final String side;

  public ShooterHoodIOTalonFX(boolean isLeft) {
    this.isLeft = isLeft;
    side = isLeft ? "Left" : "Right";

    talon =
        new TalonFX(
            isLeft ? ShooterHoodLeft.CAN_ID : ShooterHoodRight.CAN_ID,
            new CANBus(GeneralShooterHood.BUS));
    cancoder =
        new CANcoder(
            isLeft ? ShooterHoodLeft.CANCODER_ID : ShooterHoodRight.CANCODER_ID,
            new CANBus(GeneralShooterHood.BUS));

    talonConfig = talon.getConfigurator();

    slot0Configs = new Slot0Configs();
    updateSlot0Configs();

    motionMagicConfigs = new MotionMagicConfigs();
    updateMotionMagicConfigs();

    periodicUpdateSlot0 =
        () -> {
          updateSlot0Configs();

          StatusCode statusCode = talon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("ShooterHood/" + side + "/UpdateSlot0Report", statusCode);
          }
        };

    periodicUpdateMotionMagic =
        () -> {
          updateMotionMagicConfigs();

          StatusCode statusCode = talon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("ShooterHood/" + side + "/UpdateStatusCodeReport", statusCode);
          }
        };

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[8];

    statusArray[0] = talonConfig.apply(GeneralShooterHood.CONFIG);
    statusArray[1] = talonConfig.apply(slot0Configs);
    statusArray[2] = talonConfig.apply(motionMagicConfigs);
    statusArray[3] = talonConfig.apply(Constants.GENERIC_OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[4] = talonConfig.apply(Constants.GENERIC_CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[5] = talonConfig.apply(GeneralShooterHood.SOFTWARE_LIMIT_CONFIGS);
    statusArray[6] =
        talonConfig.apply(
            isLeft ? ShooterHoodLeft.FEEDBACK_CONFIGS : ShooterHoodRight.FEEDBACK_CONFIGS);
    statusArray[7] =
        cancoder
            .getConfigurator()
            .apply(isLeft ? ShooterHoodLeft.CANCODER_CONFIGS : ShooterHoodRight.CANCODER_CONFIGS);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              side + " Shooter Hood Configs",
              "Error in applying " + side + " Shooter Hood configs!"));

    Logger.recordOutput("ShooterHood/" + side + "/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    cancoderAbsolutePosition = cancoder.getAbsolutePosition();
    cancoderVelocity = cancoder.getVelocity();
    cancoderSupplyVoltage = cancoder.getSupplyVoltage();
    cancoderPositionRotations = cancoder.getPosition();

    closedLoopReferenceSlope = talon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        cancoderAbsolutePosition,
        cancoderVelocity,
        cancoderSupplyVoltage,
        cancoderPositionRotations,
        closedLoopReferenceSlope);

    voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    positionOut = new PositionVoltage(0).withEnableFOC(true).withSlot(0);
    motionMagicCurrent = new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(100.0).withSlot(0);

    BaseStatusSignal.waitForAll(0.5, cancoderAbsolutePosition);

    // this assumes that the shooter hood is starting at the bottom position
    cancoder.setPosition(0.0);
    talon.setPosition(0.0);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.cancoderConnected =
        BaseStatusSignal.refreshAll(
                cancoderAbsolutePosition,
                cancoderVelocity,
                cancoderSupplyVoltage,
                cancoderPositionRotations)
            .isOK();

    inputs.position = BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    inputs.positionRads = Units.rotationsToRadians(inputs.position);

    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();

    inputs.motionMagicVelocityTarget =
        motorPositionToRads(talon.getClosedLoopReferenceSlope().getValueAsDouble());
    inputs.motionMagicPositionTarget =
        motorPositionToRads(talon.getClosedLoopReference().getValueAsDouble());

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

    inputs.positionCancoder = Units.radiansToRotations(inputs.cancoderPositionRotations);
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
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    if (!DriverStation.isEnabled()) {
      stop();
      return;
    }

    setpointRads = clampRads(rads);
    talon.setControl(motionMagicCurrent.withPosition(radsToMotorPosition(setpointRads)));
  }

  /* Unused but nice to have */
  @Override
  public void holdPosition(double rads) {
    talon.setControl(positionOut.withPosition(radsToMotorPosition(rads)));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  private double clampRads(double rads) {
    return MathUtil.clamp(
        rads, GeneralShooterHood.MIN_POSITION_RADS, GeneralShooterHood.MAX_POSITION_RADS);
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition);
  }
}
