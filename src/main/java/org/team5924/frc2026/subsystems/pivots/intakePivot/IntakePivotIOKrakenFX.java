/*
 * IntakePivotIOKrakenFX.java
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

package org.team5924.frc2026.subsystems.pivots.intakePivot;

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

public class IntakePivotIOKrakenFX implements IntakePivotIO {
  /* Hardware */
  private final TalonFX intakePivotTalon;
  private final CANcoder intakePivotCANCoder;

  /* Configurators */
  private TalonFXConfigurator intakePivotTalonConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointRads;

  /* Gains */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP", 3.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("IntakePivot/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD", 0.07);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("IntakePivot/kS", 0.13);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("IntakePivot/kV", 0.4);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("IntakePivot/kA", 0.00);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("IntakePivot/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("IntakePivot/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("IntakePivot/MotionJerk", 0.0);

  /* Status Signals */
  private final StatusSignal<Angle> intakePivotPosition;
  private final StatusSignal<AngularVelocity> intakePivotVelocity;
  private final StatusSignal<Voltage> intakePivotAppliedVoltage;
  private final StatusSignal<Current> intakePivotSupplyCurrent;
  private final StatusSignal<Current> intakePivotTorqueCurrent;
  private final StatusSignal<Temperature> intakePivotTempCelsius;

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

  public IntakePivotIOKrakenFX() {
    intakePivotTalon =
        new TalonFX(Constants.IntakePivot.CAN_ID, new CANBus(Constants.IntakePivot.BUS));
    intakePivotCANCoder = new CANcoder(Constants.IntakePivot.CANCODER_ID);

    intakePivotTalonConfig = intakePivotTalon.getConfigurator();

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

    statusArray[0] = intakePivotTalonConfig.apply(Constants.IntakePivot.CONFIG);
    statusArray[1] = intakePivotTalonConfig.apply(slot0Configs);
    statusArray[2] = intakePivotTalonConfig.apply(motionMagicConfigs);
    statusArray[3] = intakePivotTalonConfig.apply(Constants.IntakePivot.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[4] = intakePivotTalonConfig.apply(Constants.IntakePivot.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[5] = intakePivotTalonConfig.apply(Constants.IntakePivot.FEEDBACK_CONFIGS);
    statusArray[6] =
        intakePivotCANCoder.getConfigurator().apply(Constants.IntakePivot.CANCODER_CONFIG);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING, "IntakePivot Configs", "Error in intakePivot configs!"));

    Logger.recordOutput("IntakePivot/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    intakePivotPosition = intakePivotTalon.getPosition();
    intakePivotVelocity = intakePivotTalon.getVelocity();
    intakePivotAppliedVoltage = intakePivotTalon.getMotorVoltage();
    intakePivotSupplyCurrent = intakePivotTalon.getSupplyCurrent();
    intakePivotTorqueCurrent = intakePivotTalon.getTorqueCurrent();
    intakePivotTempCelsius = intakePivotTalon.getDeviceTemp();

    cancoderAbsolutePosition = intakePivotCANCoder.getAbsolutePosition();
    cancoderVelocity = intakePivotCANCoder.getVelocity();
    cancoderSupplyVoltage = intakePivotCANCoder.getSupplyVoltage();
    cancoderPositionRotations = intakePivotCANCoder.getPosition();

    closedLoopReferenceSlope = intakePivotTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        intakePivotPosition,
        intakePivotVelocity,
        intakePivotAppliedVoltage,
        intakePivotSupplyCurrent,
        intakePivotTorqueCurrent,
        intakePivotTempCelsius,
        cancoderAbsolutePosition,
        cancoderVelocity,
        cancoderSupplyVoltage,
        cancoderPositionRotations,
        closedLoopReferenceSlope);

    voltageOut = new VoltageOut(0.0);
    positionOut = new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true).withSlot(0);
    magicMotionVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true).withSlot(0);

    BaseStatusSignal.waitForAll(0.5, cancoderAbsolutePosition);
    intakePivotCANCoder.setPosition(0.0);
    intakePivotTalon.setPosition(0.0);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.intakePivotMotorConnected =
        BaseStatusSignal.refreshAll(
                intakePivotPosition,
                intakePivotVelocity,
                intakePivotAppliedVoltage,
                intakePivotSupplyCurrent,
                intakePivotTorqueCurrent,
                intakePivotTempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.cancoderConnected =
        BaseStatusSignal.refreshAll(
                cancoderAbsolutePosition,
                cancoderVelocity,
                cancoderSupplyVoltage,
                cancoderPositionRotations)
            .isOK();

    inputs.intakePivotPosition =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                intakePivotPosition, intakePivotVelocity)
            / Constants.IntakePivot.MOTOR_TO_MECHANISM
            / Constants.IntakePivot.MOTOR_TO_CANCODER;
    inputs.intakePivotPositionRads = Units.rotationsToRadians(inputs.intakePivotPosition);

    inputs.intakePivotVelocityRadsPerSec =
        Units.rotationsToRadians(intakePivotVelocity.getValueAsDouble());
    inputs.intakePivotAppliedVoltage = intakePivotAppliedVoltage.getValueAsDouble();
    inputs.intakePivotSupplyCurrentAmps = intakePivotSupplyCurrent.getValueAsDouble();
    inputs.intakePivotTorqueCurrentAmps = intakePivotTorqueCurrent.getValueAsDouble();
    inputs.intakePivotTempCelsius = intakePivotTempCelsius.getValueAsDouble();

    inputs.motionMagicVelocityTarget =
        motorPositionToRads(intakePivotTalon.getClosedLoopReferenceSlope().getValueAsDouble());
    inputs.motionMagicPositionTarget =
        motorPositionToRads(intakePivotTalon.getClosedLoopReference().getValueAsDouble());

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

    inputs.intakePivotPositionCancoder =
        (inputs.cancoderPositionRotations) / Constants.IntakePivot.CANCODER_TO_MECHANISM;
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

          StatusCode statusCode = intakePivotTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Intake Pivot Slot 0 Configs",
                    "Error in periodically updating intake pivot Slot0 configs!"));

            Logger.recordOutput("IntakePivot/UpdateSlot0Report", statusCode);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kV);

    LoggedTunableNumber.ifChanged(
        0,
        () -> {
          motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
          motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
          motionMagicConfigs.MotionMagicJerk = motionJerk.get();

          StatusCode statusCode = intakePivotTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Intake Pivot Motion Magic Configs",
                    "Error in periodically updating intake pivot MotionMagic configs!"));

            Logger.recordOutput("IntakePivot/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void runVolts(double volts) {
    intakePivotTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    if (!DriverStation.isEnabled()) {
      stop();
      return;
    }

    setpointRads = clampRads(rads);
    intakePivotTalon.setControl(magicMotionVoltage.withPosition(radsToMotorPosition(setpointRads)));
  }

  @Override
  public void holdPosition(double rads) {
    intakePivotTalon.setControl(positionOut.withPosition(radsToMotorPosition(rads)));
  }

  @Override
  public void stop() {
    intakePivotTalon.stopMotor();
  }

  private double clampRads(double rads) {
    return MathUtil.clamp(
        rads, Constants.IntakePivot.MIN_POSITION_RADS, Constants.IntakePivot.MAX_POSITION_RADS);
  }

  private double radsToMotorPosition(double rads) { // TODO: umm double check!!
    return rads
        / Math.PI
        / 2
        * Constants.IntakePivot.MOTOR_TO_MECHANISM
        * Constants.IntakePivot.MOTOR_TO_CANCODER;
  }

  private double motorPositionToRads(double motorPosition) { // TODO: double check this too!!!
    return motorPosition
        / Constants.IntakePivot.MOTOR_TO_MECHANISM
        / Constants.IntakePivot.MOTOR_TO_CANCODER
        * Math.PI
        * 2;
  }
}
