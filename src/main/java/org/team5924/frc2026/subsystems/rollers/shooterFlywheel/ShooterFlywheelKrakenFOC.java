/*
 * ShooterFlywheelKrakenFOC.java
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

package org.team5924.frc2026.subsystems.rollers.shooterFlywheel;

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
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.Constants.GeneralShooterFlywheel;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ShooterFlywheelKrakenFOC implements ShooterFlywheelIO {
  /* Hardware */
  private final TalonFX shooterFlywheelLeaderTalon;
  private final TalonFX shooterFlywheelFollowerTalon;

  /* Configurators */
  private TalonFXConfigurator shooterFlywheelTalonLeaderConfig;
  private TalonFXConfigurator shooterFlywheelTalonFollowerConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;

  /* Gains */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("ShooterFlywheel/kP", 0.4);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("ShooterFlywheel/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("ShooterFlywheel/kD", 0.22);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("ShooterFlywheel/kS", 0.019);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("ShooterFlywheel/kV", 0.4);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("ShooterFlywheel/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("ShooterFlywheel/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("ShooterFlywheel/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("ShooterFlywheel/MotionJerk", 0.0);

  /* Status Signals */
  private final StatusSignal<Angle> shooterFlywheelPosition;
  private final StatusSignal<AngularVelocity> shooterFlywheelVelocity;
  private final StatusSignal<Voltage> shooterFlywheelAppliedVoltage;
  private final StatusSignal<Current> shooterFlywheelSupplyCurrent;
  private final StatusSignal<Current> shooterFlywheelTorqueCurrent;
  private final StatusSignal<Temperature> shooterFlywheelTempCelsius;

  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final VoltageOut voltageOut;
  private final MotionMagicVelocityVoltage motionMagicVelocity;

  private final String sideName;
  private final double reduction;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Flywheel/AtGoal")
  private boolean atGoal = false;

  public ShooterFlywheelKrakenFOC(boolean isLeft) {
    sideName = isLeft ? "Left" : "Right";
    reduction = Constants.GeneralShooterFlywheel.REDUCTION;

    shooterFlywheelLeaderTalon =
        new TalonFX(
            isLeft
                ? Constants.ShooterRollerLeaderLeft.CAN_ID
                : Constants.ShooterRollerLeaderRight.CAN_ID,
            new CANBus(Constants.GeneralShooterFlywheel.BUS));

    shooterFlywheelFollowerTalon =
        new TalonFX(
            isLeft
                ? Constants.ShooterRollerFollowerLeft.CAN_ID
                : Constants.ShooterRollerFollowerRight.CAN_ID,
            new CANBus(Constants.GeneralShooterFlywheel.BUS));

    shooterFlywheelTalonLeaderConfig = shooterFlywheelLeaderTalon.getConfigurator();
    shooterFlywheelTalonFollowerConfig = shooterFlywheelFollowerTalon.getConfigurator();

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
        shooterFlywheelTalonLeaderConfig.apply(
            isLeft
                ? Constants.ShooterRollerLeaderLeft.CONFIG
                : Constants.ShooterRollerLeaderRight.CONFIG);
    statusArray[1] =
        shooterFlywheelTalonLeaderConfig.apply(GeneralShooterFlywheel.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[2] =
        shooterFlywheelTalonLeaderConfig.apply(GeneralShooterFlywheel.OPEN_LOOP_RAMPS_CONFIGS);

    statusArray[4] = shooterFlywheelTalonLeaderConfig.apply(slot0Configs);

    statusArray[5] =
        shooterFlywheelTalonFollowerConfig.apply(
            isLeft
                ? Constants.ShooterRollerFollowerLeft.CONFIG
                : Constants.ShooterRollerFollowerRight.CONFIG);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING,
              sideName + "ShooterFlywheel Configs",
              "Error in" + sideName + " shooter flywheel configs!"));

    Logger.recordOutput("ShooterFlywheel/InitConfReport", statusArray);

    shooterFlywheelFollowerTalon.setControl(
        isLeft
            ? new Follower(Constants.ShooterRollerFollowerLeft.CAN_ID, MotorAlignmentValue.Opposed)
            : new Follower(
                Constants.ShooterRollerFollowerRight.CAN_ID, MotorAlignmentValue.Opposed));

    // Get select status signals and set update frequency
    shooterFlywheelPosition = shooterFlywheelLeaderTalon.getPosition();
    shooterFlywheelVelocity = shooterFlywheelLeaderTalon.getVelocity();
    shooterFlywheelAppliedVoltage = shooterFlywheelLeaderTalon.getMotorVoltage();
    shooterFlywheelSupplyCurrent = shooterFlywheelLeaderTalon.getSupplyCurrent();
    shooterFlywheelTorqueCurrent = shooterFlywheelLeaderTalon.getTorqueCurrent();
    shooterFlywheelTempCelsius = shooterFlywheelLeaderTalon.getDeviceTemp();

    closedLoopReferenceSlope = shooterFlywheelLeaderTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        shooterFlywheelPosition,
        shooterFlywheelVelocity,
        shooterFlywheelAppliedVoltage,
        shooterFlywheelSupplyCurrent,
        shooterFlywheelTorqueCurrent,
        shooterFlywheelTempCelsius);

    voltageOut = new VoltageOut(0.0);
    motionMagicVelocity = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    shooterFlywheelLeaderTalon.setPosition(0.0);
  }

  @Override
  public void updateInputs(ShooterFlywheelIOInputs inputs) {
    inputs.shooterFlywheelMotorConnected =
        BaseStatusSignal.refreshAll(
                shooterFlywheelPosition,
                shooterFlywheelVelocity,
                shooterFlywheelAppliedVoltage,
                shooterFlywheelSupplyCurrent,
                shooterFlywheelTorqueCurrent,
                shooterFlywheelTempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.shooterFlywheelPosition =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            shooterFlywheelPosition, shooterFlywheelVelocity);
    inputs.shooterFlywheelPositionRads = Units.rotationsToRadians(inputs.shooterFlywheelPosition);

    inputs.shooterFlywheelVelocityRadsPerSec =
        Units.rotationsToRadians(shooterFlywheelVelocity.getValueAsDouble());
    inputs.shooterFlywheelAppliedVoltage = shooterFlywheelAppliedVoltage.getValueAsDouble();
    inputs.shooterFlywheelSupplyCurrentAmps = shooterFlywheelSupplyCurrent.getValueAsDouble();
    inputs.shooterFlywheelTorqueCurrentAmps = shooterFlywheelTorqueCurrent.getValueAsDouble();
    inputs.shooterFlywheelTempCelsius = shooterFlywheelTempCelsius.getValueAsDouble();

    inputs.motionMagicVelocityTarget =
        motorPositionToRads(
            shooterFlywheelLeaderTalon.getClosedLoopReferenceSlope().getValueAsDouble());

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

          StatusCode statusCode = shooterFlywheelLeaderTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "ShooterFlywheel Slot 0 Configs",
                    "Error in periodically updating shooterFlywheel Slot0 configs!"));

            Logger.recordOutput("ShooterFlywheel/UpdateSlot0Report", statusCode);
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

          StatusCode statusCode =
              shooterFlywheelLeaderTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "ShooterFlywheel Motion Magic Configs",
                    "Error in periodically updating shooterFlywheel MotionMagic configs!"));

            Logger.recordOutput("ShooterFlywheel/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void runVolts(double volts) {
    shooterFlywheelLeaderTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocity) {
    shooterFlywheelLeaderTalon.setControl(motionMagicVelocity.withVelocity(velocity));
  }

  @Override
  public void stop() {
    shooterFlywheelLeaderTalon.stopMotor();
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads * reduction);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition / reduction);
  }
}
