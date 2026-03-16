/*
 * FlywheelIOTalonFX.java
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

package org.team5924.frc2026.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.Constants.FlywheelFollowerLeft;
import org.team5924.frc2026.Constants.FlywheelFollowerRight;
import org.team5924.frc2026.Constants.FlywheelLeaderLeft;
import org.team5924.frc2026.Constants.FlywheelLeaderRight;
import org.team5924.frc2026.Constants.GeneralFlywheel;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class FlywheelIOTalonFX implements FlywheelIO {
  /* Hardware */
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;

  /* Configurators */
  private TalonFXConfigurator leaderConfig;
  private TalonFXConfigurator followerConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointVelocityRotationsPerSec;

  /* Gains Left */
  private final LoggedTunableNumber kPLeft = new LoggedTunableNumber("Flywheel/Left/kP", 0.5);
  private final LoggedTunableNumber kILeft = new LoggedTunableNumber("Flywheel/Left/kI", 0.0);
  private final LoggedTunableNumber kDLeft = new LoggedTunableNumber("Flywheel/Left/kD", 0.0);
  private final LoggedTunableNumber kSLeft = new LoggedTunableNumber("Flywheel/Left/kS", 0.25);
  private final LoggedTunableNumber kVLeft = new LoggedTunableNumber("Flywheel/Left/kV", 0.0705);
  private final LoggedTunableNumber kALeft = new LoggedTunableNumber("Flywheel/Left/kA", 0.0);

  private final LoggedTunableNumber motionCruiseVelocityLeft =
      new LoggedTunableNumber("Flywheel/Left/MotionCruiseVelocity", 10.0);
  private final LoggedTunableNumber motionAccelerationLeft =
      new LoggedTunableNumber("Flywheel/Left/MotionAcceleration", 100.0);
  private final LoggedTunableNumber motionJerkLeft =
      new LoggedTunableNumber("Flywheel/Left/MotionJerk", 0.0);

  /* Gains Right */
  private final LoggedTunableNumber kPRight = new LoggedTunableNumber("Flywheel/Right/kP", 0.5);
  private final LoggedTunableNumber kIRight = new LoggedTunableNumber("Flywheel/Right/kI", 0.0);
  private final LoggedTunableNumber kDRight = new LoggedTunableNumber("Flywheel/Right/kD", 0.0);
  private final LoggedTunableNumber kSRight = new LoggedTunableNumber("Flywheel/Right/kS", 0.25);
  private final LoggedTunableNumber kVRight = new LoggedTunableNumber("Flywheel/Right/kV", 0.0705);
  private final LoggedTunableNumber kARight = new LoggedTunableNumber("Flywheel/Right/kA", 0.0);

  private final LoggedTunableNumber motionCruiseVelocityRight =
      new LoggedTunableNumber("Flywheel/Right/MotionCruiseVelocity", 10.0);
  private final LoggedTunableNumber motionAccelerationRight =
      new LoggedTunableNumber("Flywheel/Right/MotionAcceleration", 100.0);
  private final LoggedTunableNumber motionJerkRight =
      new LoggedTunableNumber("Flywheel/Right/MotionJerk", 0.0);

  /* Status Signals */
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageOut;
  private final MotionMagicVelocityVoltage motionMagicVelocity;

  private final Runnable periodicUpdateSlot0;
  private final Runnable periodicUpdateMotionMagic;

  private final boolean isLeft;
  private final String side;

  public FlywheelIOTalonFX(boolean isLeft) {
    this.isLeft = isLeft;
    side = isLeft ? "Left" : "Right";

    leaderTalon =
        new TalonFX(
            isLeft ? FlywheelLeaderLeft.CAN_ID : FlywheelLeaderRight.CAN_ID,
            new CANBus(GeneralFlywheel.BUS));

    followerTalon =
        new TalonFX(
            isLeft ? FlywheelFollowerLeft.CAN_ID : FlywheelFollowerRight.CAN_ID,
            new CANBus(GeneralFlywheel.BUS));

    leaderConfig = leaderTalon.getConfigurator();
    followerConfig = followerTalon.getConfigurator();

    slot0Configs = new Slot0Configs();
    updateSlot0Configs();

    motionMagicConfigs = new MotionMagicConfigs();
    updateMotionMagicConfigs();

    periodicUpdateSlot0 =
        () -> {
          updateSlot0Configs();

          StatusCode statusCode = leaderConfig.apply(slot0Configs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("Flywheel/" + side + "/UpdateSlot0Report", statusCode);
          }
        };

    periodicUpdateMotionMagic =
        () -> {
          updateMotionMagicConfigs();

          StatusCode statusCode = leaderConfig.apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Logger.recordOutput("Flywheel/" + side + "/UpdateStatusCodeReport", statusCode);
          }
        };

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[10];

    statusArray[0] =
        leaderConfig.apply(isLeft ? FlywheelLeaderLeft.CONFIG : FlywheelLeaderRight.CONFIG);
    statusArray[1] = leaderConfig.apply(Constants.GENERIC_OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[2] = leaderConfig.apply(Constants.GENERIC_CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[3] = leaderConfig.apply(GeneralFlywheel.FEEDBACK_CONFIGS);
    statusArray[4] = leaderConfig.apply(slot0Configs);
    statusArray[5] = leaderConfig.apply(motionMagicConfigs);

    statusArray[6] =
        followerConfig.apply(isLeft ? FlywheelFollowerLeft.CONFIG : FlywheelFollowerRight.CONFIG);
    statusArray[7] = followerConfig.apply(Constants.GENERIC_OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[8] = followerConfig.apply(Constants.GENERIC_CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[9] = followerConfig.apply(GeneralFlywheel.FEEDBACK_CONFIGS);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              side + "Flywheel Configs",
              "Error in " + side + " shooter flywheel configs!"));

    Logger.recordOutput("Flywheel/" + side + "/InitConfReport", statusArray);

    followerTalon.setControl(
        isLeft
            ? new Follower(FlywheelLeaderLeft.CAN_ID, MotorAlignmentValue.Opposed)
            : new Follower(FlywheelLeaderRight.CAN_ID, MotorAlignmentValue.Opposed));

    // Get select status signals and set update frequency
    position = leaderTalon.getPosition();
    velocity = leaderTalon.getVelocity();
    appliedVoltage = leaderTalon.getMotorVoltage();
    supplyCurrent = leaderTalon.getSupplyCurrent();
    torqueCurrent = leaderTalon.getTorqueCurrent();
    tempCelsius = leaderTalon.getDeviceTemp();

    closedLoopReferenceSlope = leaderTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);

    voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    motionMagicVelocity = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    leaderTalon.setPosition(0.0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
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

    inputs.position = BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    inputs.positionRads = Units.rotationsToRadians(inputs.position);

    inputs.velocityRotationsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();

    inputs.motionMagicVelocityTarget =
        motorPositionToRads(leaderTalon.getClosedLoopReferenceSlope().getValueAsDouble());

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;

    inputs.setpointVelocityRotationsPerSec = setpointVelocityRotationsPerSec;
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
    leaderTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocityRotationsPerSec) {
    setpointVelocityRotationsPerSec = velocityRotationsPerSec;
    leaderTalon.setControl(motionMagicVelocity.withVelocity(setpointVelocityRotationsPerSec));
  }

  @Override
  public void stop() {
    setpointVelocityRotationsPerSec = 0.0;
    leaderTalon.stopMotor();
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition);
  }
}
