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
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
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

  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  private double setpoint;

  private StatusSignal<Double> closedLoopReferenceSlope;
  double prevClosedLoopReferenceSlope = 0.0;
  double prevReferenceSlopeTimestamp = 0.0;

  private final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", 0.00);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.13);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.4);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 7);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", 0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.07);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Turret/kG", 0.33);

  private final LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Turret/MotionAcceleration", 400);
  private final LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Turret/MotionCruiseVelocity", 400);
  private final LoggedTunableNumber motionJerk = new LoggedTunableNumber("Turret/MotionJerk", 1000);

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);
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

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Constants.Turret.REDUCTION;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.RotorToSensorRatio = Constants.Turret.REDUCTION;

    turretTalonConfig = turretTalon.getConfigurator();

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[6];

    statusArray[0] = turretTalonConfig.apply(Constants.Turret.CONFIG);
    statusArray[2] = turretTalonConfig.apply(slot0Configs);
    statusArray[3] = turretTalonConfig.apply(motionMagicConfigs);
    statusArray[4] = turretTalonConfig.apply(openLoopRampsConfigs);
    statusArray[5] = turretTalonConfig.apply(closedLoopRampsConfigs);
    statusArray[6] = turretTalonConfig.apply(feedbackConfigs);

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;
    Logger.recordOutput("Turret/InitConfReport", statusArray);

    // Get select status signals and set update frequency
    turretPosition = turretTalon.getPosition();
    turretVelocity = turretTalon.getVelocity();
    turretAppliedVoltage = turretTalon.getMotorVoltage();
    turretSupplyCurrent = turretTalon.getSupplyCurrent();
    turretTorqueCurrent = turretTalon.getTorqueCurrent();
    turretTempCelsius = turretTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        turretPosition,
        turretVelocity,
        turretAppliedVoltage,
        turretSupplyCurrent,
        turretTorqueCurrent,
        turretTempCelsius,
        closedLoopReferenceSlope);

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
    inputs.turretPositionRads =
        Units.rotationsToRadians(turretPosition.getValueAsDouble()) / Constants.Turret.REDUCTION;
    inputs.turretVelocityRadsPerSec =
        Units.rotationsToRadians(turretVelocity.getValueAsDouble()) / Constants.Turret.REDUCTION;
    inputs.turretAppliedVoltage = turretAppliedVoltage.getValueAsDouble();
    inputs.turretSupplyCurrentAmps = turretSupplyCurrent.getValueAsDouble();
    inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
    inputs.turretTempCelsius = turretTempCelsius.getValueAsDouble();

    inputs.setpointMeters = setpoint;

    double currentTime = closedLoopReferenceSlope.getTimestamp().getTime();
    double timeDiff = currentTime - prevReferenceSlopeTimestamp;
    if (timeDiff > 0.0) {
      inputs.acceleration =
          (inputs.motionMagicVelocityTarget - prevClosedLoopReferenceSlope) / timeDiff;
    }
    prevClosedLoopReferenceSlope = inputs.motionMagicVelocityTarget;
    prevReferenceSlopeTimestamp = currentTime;

    // inputs.minSoftStop = turretCANdi.getS1Closed().getValue();
    // inputs.maxSoftStop = turretCANdi.getS2Closed().getValue();
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
}
