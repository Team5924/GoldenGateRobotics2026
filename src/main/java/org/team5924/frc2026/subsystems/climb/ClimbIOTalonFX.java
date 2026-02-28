/*
 * ClimbIOTalonFX.java
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

package org.team5924.frc2026.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX climbTalon;
  private final CANcoder climbCANCoder;

  private final TalonFXConfigurator climbTalonConfig;

  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpointRads;

  private final LoggedTunableNumber kA = new LoggedTunableNumber("Climb/kA", 0.00);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS", 0.13);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV", 0.4);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", 6.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD", 0.07);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Climb/kG", 0.33);

  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Climb/MotionCruiseVelocity", 90.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Climb/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionJerk = new LoggedTunableNumber("Turret/MotionJerk", 0.0);

  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVoltage;
  private final StatusSignal<Current> climbSupplyCurrent;
  private final StatusSignal<Current> climbTorqueCurrent;
  private final StatusSignal<Temperature> climbTempCelsius;

  private final StatusSignal<Angle> cancoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private final StatusSignal<Voltage> cancoderSupplyVoltage;
  private final StatusSignal<Angle> cancoderPositionRotations;

  private final StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final double cancoderToMechanism;
  private final double motorToMechanism;
  private final double minPositionRads;
  private final double maxPositionRads;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut;
  private final PositionVoltage positionOut;
  private final MotionMagicVoltage motionMagicVoltage;

  public ClimbIOTalonFX() {
    cancoderToMechanism = Constants.Climb.CANCODER_TO_MECHANISM;
    motorToMechanism = Constants.Climb.MOTOR_TO_MECHANISM;
    minPositionRads = Constants.Climb.MIN_POSITION_MULTI;
    maxPositionRads = Constants.Climb.MAX_POSITION_MULTI;

    climbTalon = new TalonFX(Constants.Climb.CAN_ID, new CANBus(Constants.Climb.BUS));
    climbCANCoder = new CANcoder(Constants.Climb.CANCODER_ID, new CANBus(Constants.Climb.BUS));
    
    climbTalonConfig = climbTalon.getConfigurator();

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kG = kG.get();

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[8];

    statusArray[0] = climbTalonConfig.apply(Constants.Climb.CONFIG);
    statusArray[1] = climbTalonConfig.apply(Constants.Climb.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[2] = climbTalonConfig.apply(Constants.Climb.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[3] = climbTalonConfig.apply(Constants.Climb.FEEDBACK_CONFIGS);
    statusArray[4] = climbTalonConfig.apply(Constants.Climb.FEEDBACK_CONFIGS);
    statusArray[5] = climbTalonConfig.apply(motionMagicConfigs);
    statusArray[6] = climbTalonConfig.apply(slot0Configs);
    statusArray[7] =
        climbCANCoder
            .getConfigurator()
            .apply(new CANcoderConfiguration().withMagnetSensor(Constants.Climb.CANCODER_CONFIG));

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(NotificationLevel.WARNING, "Climb Configs", "Error in climb configs!"));

    Logger.recordOutput("Turret/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    climbPosition = climbTalon.getPosition();
    climbVelocity = climbTalon.getVelocity();
    climbAppliedVoltage = climbTalon.getMotorVoltage();
    climbSupplyCurrent = climbTalon.getSupplyCurrent();
    climbTorqueCurrent = climbTalon.getTorqueCurrent();
    climbTempCelsius = climbTalon.getDeviceTemp();

    cancoderAbsolutePosition = climbCANCoder.getAbsolutePosition();
    cancoderVelocity = climbCANCoder.getVelocity();
    cancoderSupplyVoltage = climbCANCoder.getSupplyVoltage();
    cancoderPositionRotations = climbCANCoder.getPosition();

    closedLoopReferenceSlope = climbTalon.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        climbPosition,
        climbVelocity,
        climbAppliedVoltage,
        climbSupplyCurrent,
        climbTorqueCurrent,
        climbTempCelsius,
        cancoderAbsolutePosition,
        cancoderVelocity,
        cancoderSupplyVoltage,
        cancoderPositionRotations,
        closedLoopReferenceSlope);

    voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
    positionOut = new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);
    motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true).withSlot(0);

    BaseStatusSignal.waitForAll(0.5, cancoderAbsolutePosition);
    climbTalon.setPosition(0);
    climbCANCoder.setPosition(0.0); // TODO: Check if this value will always be zero
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.climbMotorConnected =
        BaseStatusSignal.refreshAll(
                climbPosition,
                climbVelocity,
                climbAppliedVoltage,
                climbSupplyCurrent,
                climbTorqueCurrent,
                climbTempCelsius,
                closedLoopReferenceSlope)
            .isOK();

    inputs.cancoderConnected =
        BaseStatusSignal.refreshAll(
                cancoderAbsolutePosition,
                cancoderVelocity,
                cancoderSupplyVoltage,
                cancoderPositionRotations)
            .isOK();
    inputs.climbPosition =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(climbPosition, climbVelocity);
    inputs.climbPositionRads = Units.rotationsToRadians(climbPosition.getValueAsDouble());
    inputs.climbVelocityRadsPerSec = Units.rotationsToRadians(climbVelocity.getValueAsDouble());
    inputs.climbAppliedVoltage = climbAppliedVoltage.getValueAsDouble();
    inputs.climbSupplyCurrentAmps = climbSupplyCurrent.getValueAsDouble();
    inputs.climbTorqueCurrentAmps = climbTorqueCurrent.getValueAsDouble();
    inputs.climbTempCelsius = climbTempCelsius.getValueAsDouble();

    inputs.motionMagicVelocityTarget =
        motorPositionToRads(climbTalon.getClosedLoopReferenceSlope().getValueAsDouble());
    inputs.motionMagicPositionTarget =
        motorPositionToRads(climbTalon.getClosedLoopReference().getValueAsDouble());

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

    inputs.climbPositionCancoder =
        Units.rotationsToRadians(inputs.cancoderPositionRotations) / cancoderToMechanism;
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
          slot0Configs.kG = kG.get();

          StatusCode statusCode = climbTalon.getConfigurator().apply(slot0Configs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Climb Slot 0 Configs",
                    "Error in periodically updating climb Slot0 configs!"));

            Logger.recordOutput("Climb/UpdateSlot0Report", statusCode);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG);

      LoggedTunableNumber.ifChanged(
        0,
        () -> {
          motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
          motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
          motionMagicConfigs.MotionMagicJerk = motionJerk.get();

          StatusCode statusCode = climbTalon.getConfigurator().apply(motionMagicConfigs);
          if (!statusCode.isOK()) {
            Elastic.sendNotification(
                new Notification(
                    NotificationLevel.WARNING,
                    "Climb Motion Magic Configs",
                    "Error in periodically updating climb MotionMagic configs!"));

            Logger.recordOutput("Climb/UpdateStatusCodeReport", statusCode);
          }
        },
        motionAcceleration,
        motionCruiseVelocity,
        motionJerk);
  }

  @Override
  public void runVolts(double volts) {
    climbTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    climbTalon.stopMotor();
  }

  @Override
  public void setPosition(double rads) {
    setpointRads = clampRads(rads);
    climbTalon.setControl(motionMagicVoltage.withPosition(radsToMotorPosition(setpointRads)));
  }

  @Override
  public void holdPosition(double rads) {
    climbTalon.setControl(positionOut.withPosition(radsToMotorPosition(rads)));
  }

  private double clampRads(double rads) {
    return MathUtil.clamp(rads, minPositionRads, maxPositionRads);
  }

  private double radsToMotorPosition(double rads) {
    return Units.radiansToRotations(rads * motorToMechanism);
  }

  private double motorPositionToRads(double motorPosition) {
    return Units.rotationsToRadians(motorPosition / motorToMechanism);
  }
}
