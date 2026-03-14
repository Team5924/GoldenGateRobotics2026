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

  /* Gains */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.4);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.22);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.019);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.4);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Flywheel/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Flywheel/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("Flywheel/MotionJerk", 0.0);

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

  private final String sideName;

  public FlywheelIOTalonFX(boolean isLeft) {
    sideName = isLeft ? "Left" : "Right";

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
    StatusCode[] statusArray = new StatusCode[6];

    statusArray[0] =
        leaderConfig.apply(isLeft ? FlywheelLeaderLeft.CONFIG : FlywheelLeaderRight.CONFIG);
    statusArray[1] = leaderConfig.apply(Constants.GENERIC_OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[2] = leaderConfig.apply(Constants.GENERIC_CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[3] = leaderConfig.apply(GeneralFlywheel.FEEDBACK_CONFIGS);

    statusArray[4] = leaderConfig.apply(slot0Configs);

    statusArray[5] =
        followerConfig.apply(isLeft ? FlywheelFollowerLeft.CONFIG : FlywheelFollowerRight.CONFIG);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              sideName + "Flywheel Configs",
              "Error in" + sideName + " shooter flywheel configs!"));

    Logger.recordOutput("Flywheel/" + sideName + "/InitConfReport", statusArray);

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

    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
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
  }

  @Override
  public void periodicUpdates() {
    updateLoggedTunableNumbers();
  }

  private void updateLoggedTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          slot0Configs.kP = kP.get();
          slot0Configs.kI = kI.get();
          slot0Configs.kD = kD.get();
          slot0Configs.kS = kS.get();
          slot0Configs.kV = kV.get();
          slot0Configs.kA = kA.get();

          StatusCode statusCode = leaderTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Flywheel Slot 0 Configs",
                    "Error in periodically updating flywheel Slot0 configs!"));

            Logger.recordOutput("Flywheel/UpdateSlot0Report", statusCode);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> {
          motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
          motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
          motionMagicConfigs.MotionMagicJerk = motionJerk.get();

          StatusCode statusCode = leaderTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Flywheel Motion Magic Configs",
                    "Error in periodically updating flywheel MotionMagic configs!"));

            Logger.recordOutput("Flywheel/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void runVolts(double volts) {
    leaderTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocity) {
    leaderTalon.setControl(motionMagicVelocity.withVelocity(velocity));
  }

  @Override
  public void stop() {
    leaderTalon.stopMotor();
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition);
  }
}
