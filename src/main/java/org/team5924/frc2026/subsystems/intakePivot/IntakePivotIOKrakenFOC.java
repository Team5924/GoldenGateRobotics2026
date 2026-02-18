/*
 * IntakePivotIOKrakenFOC.java
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

package org.team5924.frc2026.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class IntakePivotIOKrakenFOC implements IntakePivotIO {
  private final TalonFX intakePivotKraken;

  private final Alert initialMotorConfigAlert =
      new Alert(
          "Initial intake pivot motor config error! Restart robot code to clear.",
          Alert.AlertType.kError);
  private final Alert updateMotorConfigAlert =
      new Alert("Update intake pivot motor config error!", Alert.AlertType.kWarning);

  private final StatusSignal<Angle> intakePivotPosition;
  private final StatusSignal<AngularVelocity> intakePivotVelocity;
  private final StatusSignal<Voltage> intakePivotAppliedVolts;
  private final StatusSignal<Current> intakePivotSupplyCurrent;
  private final StatusSignal<Current> intakePivotTorqueCurrent;
  private final StatusSignal<Temperature> intakePivotTempCelsius;

  private StatusSignal<Double> closedLoopReferenceSlope;
  double prevClosedLoopReferenceSlope = 0.0;
  double prevReferenceSlopeTimestamp = 0.0;
  double setpoint;

  // TODO:Update these
  LoggedTunableNumber intakePivotMotorkP = new LoggedTunableNumber("IntakePivot/MotorkP", 5);
  LoggedTunableNumber intakePivotMotorkI = new LoggedTunableNumber("IntakePivot/MotorkI", 0);
  LoggedTunableNumber intakePivotMotorkD = new LoggedTunableNumber("IntakePivot/MotorkD", 0);
  LoggedTunableNumber intakePivotMotorkS = new LoggedTunableNumber("IntakePivot/MotorkS", 0);
  LoggedTunableNumber intakePivotMotorkG = new LoggedTunableNumber("IntakePivot/MotorkG", 0.11);
  LoggedTunableNumber intakePivotMotorkV = new LoggedTunableNumber("IntakePivot/MotorkV", 10.08);
  LoggedTunableNumber intakePivotMotorkA = new LoggedTunableNumber("IntakePivot/MotorkA", 0.03);

  // TODO:Update these too
  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("IntakePivot/MotionAcceleration", 400);
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("IntakePivot/MotionCruiseVelocity", 400);
  LoggedTunableNumber motionJerk = new LoggedTunableNumber("IntakePivot/MotionJerk", 1000);

  private final VoltageOut voltageControl =
      new VoltageOut(0).withUpdateFreqHz(0.0).withEnableFOC(true);
  private final MotionMagicVoltage magicMotionVoltage =
      new MotionMagicVoltage(0).withEnableFOC(true);

  private TalonFXConfigurator intakePivotKrakenConfig;
  private MotionMagicConfigs motionMagicConfigs;
  private Slot0Configs controllerConfigs;

  public IntakePivotIOKrakenFOC() {
    intakePivotKraken =
        new TalonFX(
            Constants.IntakePivot.INTAKE_PIVOT_CAN_ID, Constants.IntakePivot.INTAKE_PIVOT_BUS);

    this.intakePivotKrakenConfig = intakePivotKraken.getConfigurator();
    final TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    krakenConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.IntakePivot.INTAKE_PIVOT_CURRENT_LIMIT;
    krakenConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    krakenConfig.MotorOutput.Inverted = Constants.IntakePivot.INTAKE_PIVOT_INVERT;
    krakenConfig.MotorOutput.NeutralMode = Constants.IntakePivot.INTAKE_PIVOT_BRAKE;
    krakenConfig.Feedback.SensorToMechanismRatio = Constants.IntakePivot.INTAKE_PIVOT_REDUCTION;

    controllerConfigs = new Slot0Configs();
    controllerConfigs.kP = intakePivotMotorkP.getAsDouble();
    controllerConfigs.kI = intakePivotMotorkI.getAsDouble();
    controllerConfigs.kD = intakePivotMotorkD.getAsDouble();
    controllerConfigs.kS = intakePivotMotorkS.getAsDouble();
    controllerConfigs.kG = intakePivotMotorkG.getAsDouble();
    controllerConfigs.kV = intakePivotMotorkV.getAsDouble();
    controllerConfigs.kA = intakePivotMotorkA.getAsDouble();

    intakePivotPosition = intakePivotKraken.getPosition();
    intakePivotVelocity = intakePivotKraken.getVelocity();
    intakePivotAppliedVolts = intakePivotKraken.getMotorVoltage();
    intakePivotSupplyCurrent = intakePivotKraken.getSupplyCurrent();
    intakePivotTorqueCurrent = intakePivotKraken.getTorqueCurrent();
    intakePivotTempCelsius = intakePivotKraken.getDeviceTemp();

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Constants.IntakePivot.INTAKE_PIVOT_REDUCTION;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.RotorToSensorRatio = 1.0;

    StatusCode[] statusArray = new StatusCode[6];

    statusArray[0] = intakePivotKrakenConfig.apply(krakenConfig);
    statusArray[1] = intakePivotKrakenConfig.apply(controllerConfigs);
    statusArray[2] = intakePivotKrakenConfig.apply(motionMagicConfigs);
    statusArray[3] = intakePivotKrakenConfig.apply(openLoopRampsConfigs);
    statusArray[4] = intakePivotKrakenConfig.apply(closedLoopRampsConfigs);
    statusArray[5] = intakePivotKrakenConfig.apply(feedbackConfigs);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;
    initialMotorConfigAlert.set(isErrorPresent);

    closedLoopReferenceSlope = intakePivotKraken.getClosedLoopReference();

    intakePivotKraken.setPosition(0.0);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs input) {
    input.intakePivotMotorConnected =
        BaseStatusSignal.refreshAll(
                intakePivotPosition,
                intakePivotVelocity,
                intakePivotAppliedVolts,
                intakePivotSupplyCurrent,
                intakePivotTorqueCurrent,
                intakePivotTempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    input.intakePivotPositionRads = intakePivotPosition.getValue().in(Radians);
    input.intakePivotVelocityRadsPerSec = intakePivotVelocity.getValue().in(RadiansPerSecond);
    input.intakePivotAppliedVolts = intakePivotAppliedVolts.getValue().in(Volts);
    input.intakePivotSupplyCurrentAmps = intakePivotSupplyCurrent.getValue().in(Amps);
    input.intakePivotTorqueCurrentAmps = intakePivotTorqueCurrent.getValue().in(Amps);
    input.intakePivotTempCelsius = intakePivotTempCelsius.getValue().in(Celsius);

    input.motionMagicVelocityTarget = (intakePivotKraken.getClosedLoopReferenceSlope().getValue());
    input.motionMagicPositionTarget = intakePivotKraken.getClosedLoopReference().getValue();

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      input.acceleration =
          (input.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }

    prevClosedLoopReferenceSlope = input.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;

    input.intakePivotSetpoint = setpoint;
  }

  @Override
  public void setVoltage(double volts) {
    if (atSoftStop(volts)) {
      intakePivotKraken.setControl(voltageControl.withOutput(0));
      return;
    }
    intakePivotKraken.setControl(voltageControl.withOutput(volts));
  }

  public boolean atSoftStop(double volts) {
    double currentAngle =
        Units.rotationsToRadians(intakePivotPosition.getValueAsDouble())
            / Constants.IntakePivot.INTAKE_PIVOT_REDUCTION;
    return (currentAngle >= Constants.IntakePivot.INTAKE_PIVOT_MAX_RADS && volts > 0)
        || (currentAngle <= Constants.IntakePivot.INTAKE_PIVOT_MIN_RADS && volts < 0);
  }

  @Override
  public void setPosition(double rads) {
    setpoint = rads;
    intakePivotKraken.setControl(magicMotionVoltage.withPosition(Units.radiansToRotations(rads)));
  }

  public void updateTunableNumbers() {
    if (intakePivotMotorkA.hasChanged(hashCode())
        || intakePivotMotorkS.hasChanged(hashCode())
        || intakePivotMotorkV.hasChanged(hashCode())
        || intakePivotMotorkP.hasChanged(hashCode())
        || intakePivotMotorkI.hasChanged(hashCode())
        || intakePivotMotorkD.hasChanged(hashCode())
        || intakePivotMotorkG.hasChanged(hashCode())
        || motionAcceleration.hasChanged(hashCode())
        || motionCruiseVelocity.hasChanged(hashCode())) {
      controllerConfigs.kA = intakePivotMotorkA.get();
      controllerConfigs.kS = intakePivotMotorkS.get();
      controllerConfigs.kV = intakePivotMotorkV.get();
      controllerConfigs.kP = intakePivotMotorkP.get();
      controllerConfigs.kI = intakePivotMotorkI.get();
      controllerConfigs.kD = intakePivotMotorkD.get();
      controllerConfigs.kG = intakePivotMotorkG.get();

      motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

      StatusCode[] statusArray = new StatusCode[2];

      statusArray[0] = intakePivotKrakenConfig.apply(controllerConfigs);
      statusArray[1] = intakePivotKrakenConfig.apply(motionMagicConfigs);

      boolean isErrorPresent = false;
      for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;
      updateMotorConfigAlert.set(isErrorPresent);
    }
  }

  @Override
  public void setSoftStopOn() {}

  @Override
  public void setSoftStopOff() {}

  // Stop
  @Override
  public void stop() {
    intakePivotKraken.stopMotor();
  }
}
