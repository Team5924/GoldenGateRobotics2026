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
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ShooterHoodIOTalonFX implements ShooterHoodIO {
  /* Hardware */
  private final TalonFX shooterHoodTalon;
  private final CANcoder shooterHoodCANCoder;

  /* Configurators */
  private TalonFXConfigurator shooterHoodTalonConfig;

  /* Configs */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointRads;

  /* Gains Left */
  private final LoggedTunableNumber kPLeft = new LoggedTunableNumber("ShooterHood/Left/kP", 1.0);
  private final LoggedTunableNumber kILeft = new LoggedTunableNumber("ShooterHood/Left/kI", 0.0);
  private final LoggedTunableNumber kDLeft = new LoggedTunableNumber("ShooterHood/Left/kD", 0.00);
  private final LoggedTunableNumber kSLeft = new LoggedTunableNumber("ShooterHood/Left/kS", 0.0);
  private final LoggedTunableNumber kVLeft = new LoggedTunableNumber("ShooterHood/Left/kV", 0.0);
  private final LoggedTunableNumber kALeft = new LoggedTunableNumber("ShooterHood/Left/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocityLeft =
      new LoggedTunableNumber("ShooterHood/Left/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAccelerationLeft =
      new LoggedTunableNumber("ShooterHood/Left/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerkLeft =
      new LoggedTunableNumber("ShooterHood/Left/MotionJerk", 0.0);

  /* Gains Right */
  private final LoggedTunableNumber kPRight = new LoggedTunableNumber("ShooterHood/Right/kP", 1.0);
  private final LoggedTunableNumber kIRight = new LoggedTunableNumber("ShooterHood/Right/kI", 0.0);
  private final LoggedTunableNumber kDRight = new LoggedTunableNumber("ShooterHood/Right/kD", 0.00);
  private final LoggedTunableNumber kSRight = new LoggedTunableNumber("ShooterHood/Right/kS", 0.0);
  private final LoggedTunableNumber kVRight = new LoggedTunableNumber("ShooterHood/Right/kV", 0.0);
  private final LoggedTunableNumber kARight = new LoggedTunableNumber("ShooterHood/Right/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocityRight =
      new LoggedTunableNumber("ShooterHood/Right/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAccelerationRight =
      new LoggedTunableNumber("ShooterHood/Right/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerkRight =
      new LoggedTunableNumber("ShooterHood/Right/MotionJerk", 0.0);

  /* Status Signals */
  private final StatusSignal<Angle> shooterHoodPosition;
  private final StatusSignal<AngularVelocity> shooterHoodVelocity;
  private final StatusSignal<Voltage> shooterHoodAppliedVoltage;
  private final StatusSignal<Current> shooterHoodSupplyCurrent;
  private final StatusSignal<Current> shooterHoodTorqueCurrent;
  private final StatusSignal<Temperature> shooterHoodTempCelsius;

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

    shooterHoodTalon =
        new TalonFX(
            isLeft ? Constants.ShooterHoodLeft.CAN_ID : Constants.ShooterHoodRight.CAN_ID,
            new CANBus(Constants.GeneralShooterHood.BUS));
    shooterHoodCANCoder =
        new CANcoder(
            isLeft
                ? Constants.ShooterHoodLeft.CANCODER_ID
                : Constants.ShooterHoodRight.CANCODER_ID);

    shooterHoodTalonConfig = shooterHoodTalon.getConfigurator();

    slot0Configs = new Slot0Configs();
    updateSlot0Configs();

    motionMagicConfigs = new MotionMagicConfigs();
    updateMotionMagicConfigs();

    periodicUpdateSlot0 =
        () -> {
          updateSlot0Configs();

          StatusCode statusCode = shooterHoodTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("ShooterHood/" + side + "/UpdateSlot0Report", statusCode);
          }
        };

    periodicUpdateMotionMagic =
        () -> {
          updateMotionMagicConfigs();

          StatusCode statusCode = shooterHoodTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("ShooterHood/Right/UpdateStatusCodeReport", statusCode);
          }
        };

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[7];

    statusArray[0] = shooterHoodTalonConfig.apply(Constants.GeneralShooterHood.CONFIG);
    statusArray[1] = shooterHoodTalonConfig.apply(slot0Configs);
    statusArray[2] =
        shooterHoodTalonConfig.apply(Constants.GeneralShooterHood.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[3] =
        shooterHoodTalonConfig.apply(Constants.GeneralShooterHood.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[4] =
        shooterHoodTalonConfig.apply(
            isLeft
                ? Constants.ShooterHoodLeft.FEEDBACK_CONFIGS
                : Constants.ShooterHoodRight.FEEDBACK_CONFIGS);
    statusArray[5] =
        shooterHoodTalonConfig.apply(Constants.GeneralShooterHood.SOFTWARE_LIMIT_CONFIGS);
    statusArray[6] =
        shooterHoodCANCoder
            .getConfigurator()
            .apply(
                isLeft
                    ? Constants.ShooterHoodLeft.CANCODER_CONFIG
                    : Constants.ShooterHoodRight.CANCODER_CONFIG);

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
    shooterHoodPosition = shooterHoodTalon.getPosition();
    shooterHoodVelocity = shooterHoodTalon.getVelocity();
    shooterHoodAppliedVoltage = shooterHoodTalon.getMotorVoltage();
    shooterHoodSupplyCurrent = shooterHoodTalon.getSupplyCurrent();
    shooterHoodTorqueCurrent = shooterHoodTalon.getTorqueCurrent();
    shooterHoodTempCelsius = shooterHoodTalon.getDeviceTemp();

    cancoderAbsolutePosition = shooterHoodCANCoder.getAbsolutePosition();
    cancoderVelocity = shooterHoodCANCoder.getVelocity();
    cancoderSupplyVoltage = shooterHoodCANCoder.getSupplyVoltage();
    cancoderPositionRotations = shooterHoodCANCoder.getPosition();

    closedLoopReferenceSlope = shooterHoodTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        shooterHoodPosition,
        shooterHoodVelocity,
        shooterHoodAppliedVoltage,
        shooterHoodSupplyCurrent,
        shooterHoodTorqueCurrent,
        shooterHoodTempCelsius,
        cancoderAbsolutePosition,
        cancoderVelocity,
        cancoderSupplyVoltage,
        cancoderPositionRotations,
        closedLoopReferenceSlope);

    voltageOut = new VoltageOut(0.0);
    positionOut = new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true).withSlot(0);
    motionMagicCurrent = new MotionMagicTorqueCurrentFOC(0.0).withSlot(0);

    BaseStatusSignal.waitForAll(0.5, cancoderAbsolutePosition);

    // this assumes that the shooter hood is starting at the bottom position
    shooterHoodCANCoder.setPosition(0.0);
    shooterHoodTalon.setPosition(0.0);
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
                shooterHoodTempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.cancoderConnected =
        BaseStatusSignal.refreshAll(
                cancoderAbsolutePosition,
                cancoderVelocity,
                cancoderSupplyVoltage,
                cancoderPositionRotations)
            .isOK();

    inputs.shooterHoodPosition =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            shooterHoodPosition, shooterHoodVelocity);
    inputs.shooterHoodPositionRads = Units.rotationsToRadians(inputs.shooterHoodPosition);

    inputs.shooterHoodVelocityRadsPerSec =
        Units.rotationsToRadians(shooterHoodVelocity.getValueAsDouble());
    inputs.shooterHoodAppliedVoltage = shooterHoodAppliedVoltage.getValueAsDouble();
    inputs.shooterHoodSupplyCurrentAmps = shooterHoodSupplyCurrent.getValueAsDouble();
    inputs.shooterHoodTorqueCurrentAmps = shooterHoodTorqueCurrent.getValueAsDouble();
    inputs.shooterHoodTempCelsius = shooterHoodTempCelsius.getValueAsDouble();

    inputs.motionMagicVelocityTarget =
        motorPositionToRads(shooterHoodTalon.getClosedLoopReferenceSlope().getValueAsDouble());
    inputs.motionMagicPositionTarget =
        motorPositionToRads(shooterHoodTalon.getClosedLoopReference().getValueAsDouble());

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

    inputs.shooterHoodPositionCancoder = inputs.cancoderPositionRotations;
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
    shooterHoodTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    if (!DriverStation.isEnabled()) {
      stop();
      return;
    }

    setpointRads = clampRads(rads);
    shooterHoodTalon.setControl(motionMagicCurrent.withPosition(radsToMotorPosition(setpointRads)));
  }

  /* Unused but nice to have */
  @Override
  public void holdPosition(double rads) {
    shooterHoodTalon.setControl(positionOut.withPosition(radsToMotorPosition(rads)));
  }

  @Override
  public void stop() {
    shooterHoodTalon.stopMotor();
  }

  private double clampRads(double rads) {
    return MathUtil.clamp(
        rads,
        Constants.GeneralShooterHood.MIN_POSITION_RADS,
        Constants.GeneralShooterHood.MAX_POSITION_RADS);
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition);
  }
}
