/*
 * TurretIOTalonFX.java
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

package org.team5924.frc2026.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretTalon;
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Voltage> turretAppliedVoltage;
  private final StatusSignal<Current> turretSupplyCurrent;
  private final StatusSignal<Current> turretTorqueCurrent;
  private final StatusSignal<Temperature> turretTempCelsius;

  private TalonFXConfigurator turretTalonConfig;

  private final CANcoder cancoder;
  private final StatusSignal<Angle> cancoderPosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private final StatusSignal<Voltage> cancoderSupplyVoltage;

  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpoint;

  private StatusSignal<Double> closedLoopReferenceSlope;
  private double prevClosedLoopReferenceSlope = 0.0;
  private double prevReferenceSlopeTimestamp = 0.0;

  private final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", 0.00);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.13);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.4);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 6.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.07);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Turret/kG", 0.33);

  private final LoggedTunableNumber motionJerk = new LoggedTunableNumber("Turret/MotionJerk", 0.0);
  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Turret/MotionAcceleration", 900.0);
  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Turret/MotionCruiseVelocity", 90.0);

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true).withSlot(0);
  private final MotionMagicVoltage magicMotionVoltage;

  public TurretIOTalonFX() {
    turretTalon = new TalonFX(Constants.Turret.CAN_ID, new CANBus(Constants.Turret.BUS));

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

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Constants.Turret.REDUCTION;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.RotorToSensorRatio = 1.0;

    turretTalonConfig = turretTalon.getConfigurator();

    cancoder = new CANcoder(Constants.Turret.CANCODER_ID);

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[7];

    statusArray[0] = turretTalonConfig.apply(Constants.Turret.CONFIG);
    statusArray[2] = turretTalonConfig.apply(slot0Configs);
    statusArray[3] = turretTalonConfig.apply(motionMagicConfigs);
    statusArray[4] = turretTalonConfig.apply(Constants.Turret.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[5] = turretTalonConfig.apply(Constants.Turret.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[6] = turretTalonConfig.apply(feedbackConfigs);
    statusArray[7] = cancoder.getConfigurator().apply(Constants.Turret.CANCODER_CONFIG);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(NotificationLevel.WARNING, "Turret: Error in configs", ""));

    Logger.recordOutput("Turret/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    turretPosition = turretTalon.getPosition();
    turretVelocity = turretTalon.getVelocity();
    turretAppliedVoltage = turretTalon.getMotorVoltage();
    turretSupplyCurrent = turretTalon.getSupplyCurrent();
    turretTorqueCurrent = turretTalon.getTorqueCurrent();
    turretTempCelsius = turretTalon.getDeviceTemp();

    cancoderPosition = cancoder.getAbsolutePosition();
    cancoderVelocity = cancoder.getVelocity();
    cancoderSupplyVoltage = cancoder.getSupplyVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        turretPosition,
        turretVelocity,
        turretAppliedVoltage,
        turretSupplyCurrent,
        turretTorqueCurrent,
        turretTempCelsius,
        closedLoopReferenceSlope);

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, cancoderPosition, cancoderVelocity, cancoderSupplyVoltage);

    magicMotionVoltage = new MotionMagicVoltage(0).withEnableFOC(true);

    turretTalon.setPosition(0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretMotorConnected =
        BaseStatusSignal.refreshAll(
                turretPosition,
                turretVelocity,
                turretAppliedVoltage,
                turretSupplyCurrent,
                turretTorqueCurrent,
                turretTempCelsius)
            .isOK();

    inputs.cancoderConnected =
        BaseStatusSignal.refreshAll(cancoderPosition, cancoderVelocity, cancoderSupplyVoltage)
            .isOK();

    inputs.turretPositionRads = Units.rotationsToRadians(turretPosition.getValueAsDouble());
    inputs.turretVelocityRadsPerSec = Units.rotationsToRadians(turretVelocity.getValueAsDouble());
    inputs.turretAppliedVoltage = turretAppliedVoltage.getValueAsDouble();
    inputs.turretSupplyCurrentAmps = turretSupplyCurrent.getValueAsDouble();
    inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
    inputs.turretTempCelsius = turretTempCelsius.getValueAsDouble();

    inputs.setpointRads = setpoint;

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;

    inputs.cancoderPosition = cancoderPosition.getValueAsDouble();
    inputs.cancoderVelocity = cancoderVelocity.getValueAsDouble();
    inputs.cancoderSupplyVoltage = cancoderSupplyVoltage.getValueAsDouble();

    // inputs.minSoftStop = turretCANdi.getS1Closed().getValue();
    // inputs.maxSoftStop = turretCANdi.getS2Closed().getValue();

    updateLoggedTunableNumbers();
  }

  private void updateLoggedTunableNumbers() { // TODO: check if updated
    // LoggedTunableNumber.ifChanged(0, null, null);

    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kG = kG.get();

    // turretTalon.getConfigurator().apply(null)
  }

  @Override
  public void runVolts(double volts) {
    turretTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double rads) {
    turretTalon.setControl(positionOut.withPosition(rads));
  }

  @Override
  public void stop() {
    turretTalon.stopMotor();
  }

  private double getTurretAngleOffset() {
    BaseStatusSignal.waitForAll(10.0, cancoderPosition);
    return cancoderPosition.getValueAsDouble() / Constants.Turret.REDUCTION; // ????
  }

  @Override
  public void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {
    double setpointRadians =
        MathUtil.clamp(
            radiansFromCenter,
            Constants.Turret.MIN_POSITION_RADS,
            Constants.Turret.MAX_POSITION_RADS);
    double setpointRotations = Units.radiansToRotations(setpointRadians);
    double setpointRotor = setpointRotations / Constants.Turret.REDUCTION;
    double ffVel = Units.radiansToRotations(radsPerSecond) / Constants.Turret.REDUCTION;
    turretTalon.setControl(positionOut.withPosition(setpointRotor).withVelocity(ffVel));
    Logger.recordOutput("Turret/IO/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/radsPerSecond", radsPerSecond);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/ffVel", ffVel);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/setpointRotor", setpointRotor);
    Logger.recordOutput(
        "Turret/IO/setPositionSetpoint/radsPerSecondRotor",
        radsPerSecond / Constants.Turret.REDUCTION);
  }
}
