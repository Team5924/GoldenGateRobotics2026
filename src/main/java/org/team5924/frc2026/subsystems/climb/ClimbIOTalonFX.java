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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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

  private final LoggedTunableNumber kA = new LoggedTunableNumber("Climb/kA", 0.00);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS", 0.13);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV", 0.4);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", 6.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD", 0.07);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Climb/kG", 0.33);

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

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ClimbIOTalonFX() {
    climbTalon = new TalonFX(Constants.Climb.CAN_ID, new CANBus(Constants.Climb.BUS));
    climbCANCoder = new CANcoder(Constants.Climb.CANCODER_ID);

    slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kG = kG.get();

    climbTalonConfig = climbTalon.getConfigurator();

    // Apply Configs
    StatusCode[] statusArray = new StatusCode[6];

    statusArray[0] = climbTalonConfig.apply(Constants.Climb.CONFIG);
    statusArray[1] = climbTalonConfig.apply(slot0Configs);
    statusArray[2] = climbTalonConfig.apply(Constants.Climb.OPEN_LOOP_RAMPS_CONFIGS);
    statusArray[3] = climbTalonConfig.apply(Constants.Climb.CLOSED_LOOP_RAMPS_CONFIGS);
    statusArray[4] = climbTalonConfig.apply(Constants.Climb.FEEDBACK_CONFIGS);
    statusArray[5] =
        climbCANCoder
            .getConfigurator()
            .apply(new CANcoderConfiguration().withMagnetSensor(Constants.Climb.CANCODER_CONFIG));

    boolean isErrorPresent = false;
    for (StatusCode s : statusArray) if (!s.isOK()) isErrorPresent = true;

    if (isErrorPresent)
      Elastic.sendNotification(
          new Notification(NotificationLevel.WARNING, "Climb Configs", "Error in climb configs!"));

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
        cancoderPositionRotations);

    climbTalon.setPosition(0);
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
                climbTempCelsius)
            .isOK();

    inputs.cancoderConnected =
        BaseStatusSignal.refreshAll(
                cancoderAbsolutePosition,
                cancoderVelocity,
                cancoderSupplyVoltage,
                cancoderPositionRotations)
            .isOK();

    inputs.climbPositionRads = Units.rotationsToRadians(climbPosition.getValueAsDouble());
    inputs.climbVelocityRadsPerSec = Units.rotationsToRadians(climbVelocity.getValueAsDouble());
    inputs.climbAppliedVoltage = climbAppliedVoltage.getValueAsDouble();
    inputs.climbSupplyCurrentAmps = climbSupplyCurrent.getValueAsDouble();
    inputs.climbTorqueCurrentAmps = climbTorqueCurrent.getValueAsDouble();
    inputs.climbTempCelsius = climbTempCelsius.getValueAsDouble();

    inputs.cancoderAbsolutePosition = cancoderAbsolutePosition.getValueAsDouble();
    inputs.cancoderVelocity = cancoderVelocity.getValueAsDouble();
    inputs.cancoderSupplyVoltage = cancoderSupplyVoltage.getValueAsDouble();
    inputs.cancoderPositionRotations = cancoderPositionRotations.getValueAsDouble();

    inputs.climbPositionCancoder =
        Units.rotationsToRadians(inputs.cancoderPositionRotations)
            / Constants.Climb.CANCODER_TO_MECHANISM;
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
          climbTalon.getConfigurator().apply(slot0Configs);
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG);
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
    climbTalon.setControl(positionOut.withPosition(Units.radiansToRotations(rads)));
  }
}
