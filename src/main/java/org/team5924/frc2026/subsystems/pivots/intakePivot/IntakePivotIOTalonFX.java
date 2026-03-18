/*
 * IntakePivotIOTalonFX.java
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
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot.IntakePivotState;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  /* Hardware */
  private final TalonFX talon;

  /* Configurators */
  private TalonFXConfigurator talonConfig;

  /* Configs  */
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointRads;

  /* Gains */
  private final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP", 40.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("IntakePivot/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD", 0.5);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("IntakePivot/kS", .2);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("IntakePivot/kV", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("IntakePivot/kG", 2.8);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("IntakePivot/kA", 0.0);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("IntakePivot/MotionCruiseVelocity", 10.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("IntakePivot/MotionAcceleration", 100.0);
  private final LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("IntakePivot/MotionJerk", 0.0);

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

  private final TorqueCurrentFOC currentOut;
  private final PositionVoltage positionOut;
  private final MotionMagicTorqueCurrentFOC motionMagicCurrent;

  public IntakePivotIOTalonFX() {
    talon = new TalonFX(Constants.IntakePivot.CAN_ID, new CANBus(Constants.IntakePivot.BUS));

    talonConfig = talon.getConfigurator();

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kG = kG.get();
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[7];

    statusArray[0] = talonConfig.apply(Constants.IntakePivot.CONFIG);
    statusArray[1] = talonConfig.apply(slot0Configs);
    statusArray[2] = talonConfig.apply(motionMagicConfigs);
    statusArray[3] = talonConfig.apply(Constants.GENERIC_OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[4] = talonConfig.apply(Constants.GENERIC_CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[5] = talonConfig.apply(Constants.IntakePivot.SOFTWARE_LIMIT_CONFIGS);
    statusArray[6] = talonConfig.apply(Constants.IntakePivot.FEEDBACK_CONFIGS);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(
              NotificationLevel.WARNING, "Intake Pivot Configs", "Error in Intake Pivot configs!"));

    Logger.recordOutput("IntakePivot/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    closedLoopReferenceSlope = talon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        closedLoopReferenceSlope);

    currentOut = new TorqueCurrentFOC(0.0);
    positionOut = new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true).withSlot(0);
    motionMagicCurrent = new MotionMagicTorqueCurrentFOC(0.0).withSlot(0).withUpdateFreqHz(100);

    // assuming intake pivot starts stowed
    talon.setPosition(
        Units.radiansToRotations(IntakePivotState.PHYSICAL_STOW.getRads().getAsDouble()));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
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

    inputs.motionMagicVelocityTarget = talon.getClosedLoopReferenceSlope().getValueAsDouble();
    inputs.motionMagicPositionTarget = talon.getClosedLoopReference().getValueAsDouble();

    inputs.setpointRads = setpointRads;
    inputs.setpointPosition = Units.radiansToRotations(setpointRads);

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
          slot0Configs.kG = kG.get();
          slot0Configs.kA = kA.get();

          StatusCode statusCode = talon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "IntakePivot Slot 0 Configs",
                    "Error in periodically updating intakePivot Slot0 configs!"));

            Logger.recordOutput("IntakePivot/UpdateSlot0Report", statusCode);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kG,
        kA);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> {
          motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
          motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
          motionMagicConfigs.MotionMagicJerk = motionJerk.get();

          StatusCode statusCode = talon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "IntakePivot Motion Magic Configs",
                    "Error in periodically updating intakePivot MotionMagic configs!"));

            Logger.recordOutput("IntakePivot/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void runCurrent(double volts) {
    talon.setControl(currentOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    if (!DriverStation.isEnabled()) {
      stop();
      return;
    }

    setpointRads = clampRads(rads);
    talon.setControl(motionMagicCurrent.withPosition(radsToPosition(setpointRads)));
  }

  @Override
  public void holdPosition(double rads) {
    if (!DriverStation.isEnabled()) {
      stop();
      return;
    }

    talon.setControl(positionOut.withPosition(radsToPosition(rads)));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  private double clampRads(double rads) {
    return MathUtil.clamp(
        rads, Constants.IntakePivot.MIN_POSITION_RADS, Constants.IntakePivot.MAX_POSITION_RADS);
  }

  private double radsToPosition(double rads) {
    return Units.radiansToRotations(rads);
  }

  private double positionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition);
  }
}
