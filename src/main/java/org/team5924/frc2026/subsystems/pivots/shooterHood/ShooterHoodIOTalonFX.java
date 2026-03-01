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

  /* Gains */
  // TODO: Change values later
  private final LoggedTunableNumber kP = new LoggedTunableNumber("ShooterHood/kP", 1.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("ShooterHood/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("ShooterHood/kD", 0.00);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("ShooterHood/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("ShooterHood/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("ShooterHood/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("ShooterHood/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("ShooterHood/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("ShooterHood/MotionJerk", 0.0);

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

  private final double cancoderToMechanism;
  private final double motorToMechanism;
  private final double minPositionRads;
  private final double maxPositionRads;
  private final double mechanismRangePercent;

  public ShooterHoodIOTalonFX(boolean isLeft) {
    cancoderToMechanism =
        isLeft
            ? Constants.ShooterHoodLeft.CANCODER_TO_MECHANISM
            : Constants.ShooterHoodRight.CANCODER_TO_MECHANISM;
    motorToMechanism =
        isLeft
            ? Constants.ShooterHoodLeft.MOTOR_TO_MECHANISM
            : Constants.ShooterHoodRight.MOTOR_TO_MECHANISM;
    minPositionRads =
        isLeft
            ? Constants.ShooterHoodLeft.MIN_POSITION_RADS
            : Constants.ShooterHoodRight.MIN_POSITION_RADS;
    maxPositionRads =
        isLeft
            ? Constants.ShooterHoodLeft.MAX_POSITION_RADS
            : Constants.ShooterHoodRight.MAX_POSITION_RADS;
    mechanismRangePercent =
        isLeft
            ? Constants.ShooterHoodLeft.MECHANISM_RANGE_PERCENT
            : Constants.ShooterHoodRight.MECHANISM_RANGE_PERCENT;

    shooterHoodTalon =
        new TalonFX(
            isLeft ? Constants.ShooterHoodLeft.CAN_ID : Constants.ShooterHoodRight.CAN_ID,
            new CANBus(isLeft ? Constants.ShooterHoodLeft.BUS : Constants.ShooterHoodRight.BUS));
    shooterHoodCANCoder =
        new CANcoder(
            isLeft
                ? Constants.ShooterHoodLeft.CANCODER_ID
                : Constants.ShooterHoodRight.CANCODER_ID);

    shooterHoodTalon
        .getConfigurator()
        .apply(isLeft ? Constants.ShooterHoodLeft.CONFIG : Constants.ShooterHoodRight.CONFIG);

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

    statusArray[0] =
        shooterHoodTalonConfig.apply(
            isLeft ? Constants.ShooterHoodLeft.CONFIG : Constants.ShooterHoodRight.CONFIG);
    statusArray[1] = shooterHoodTalonConfig.apply(slot0Configs);
    statusArray[2] =
        shooterHoodTalonConfig.apply(
            isLeft
                ? Constants.ShooterHoodLeft.OPEN_LOOP_RAMPS_CONFIGS
                : Constants.ShooterHoodRight.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[3] =
        shooterHoodTalonConfig.apply(
            isLeft
                ? Constants.ShooterHoodLeft.CLOSED_LOOP_RAMPS_CONFIGS
                : Constants.ShooterHoodRight.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[4] =
        shooterHoodTalonConfig.apply(
            isLeft
                ? Constants.ShooterHoodLeft.FEEDBACK_CONFIGS
                : Constants.ShooterHoodRight.FEEDBACK_CONFIGS);
    statusArray[5] =
        shooterHoodTalonConfig.apply(
            isLeft
                ? Constants.ShooterHoodLeft.SOFTWARE_LIMIT_CONFIGS
                : Constants.ShooterHoodRight.SOFTWARE_LIMIT_CONFIGS);
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
              NotificationLevel.WARNING, "Shooter Hood Configs", "Error in shooter hood configs!"));

    Logger.recordOutput("ShooterHood/InitConfReport", statusArray);

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

    inputs.shooterHoodPositionCancoder = inputs.cancoderPositionRotations / cancoderToMechanism;
  }

  @Override
  public void periodicUpdates() {
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

          StatusCode statusCode = shooterHoodTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Shooter Hood Slot 0 Configs",
                    "Error in periodically updating shooter hood Slot0 configs!"));
            Logger.recordOutput("ShooterHood/UpdateSlot0Report", statusCode);
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

          StatusCode statusCode = shooterHoodTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Shooter Hood Motion Magic Configs",
                    "Error in periodically updating shooter hood MotionMagic configs!"));

            Logger.recordOutput("ShooterHood/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
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
    return MathUtil.clamp(rads, minPositionRads, maxPositionRads);
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads) * motorToMechanism * mechanismRangePercent;
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition) / motorToMechanism / mechanismRangePercent;
  }
}
