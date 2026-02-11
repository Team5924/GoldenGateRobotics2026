/*
 * ArmPracticeIOTalonFX.java
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

package org.team5924.frc2026.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import org.team5924.frc2026.Constants;

public class ArmPracticeIOTalonFX implements ArmPracticeIO {
  private final TalonFX armPracticeTalon;
  private final StatusSignal<Angle> armPracticePosition;
  private final StatusSignal<AngularVelocity> armPracticeVelocity;
  private final StatusSignal<Voltage> armPracticeAppliedVoltage;
  private final StatusSignal<Current> armPracticeSupplyCurrent;
  private final StatusSignal<Current> armPracticeTorqueCurrent;
  private final StatusSignal<Temperature> armPracticeTempCelsius;

  private final DigitalInput beamBreak = new DigitalInput(0);

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageout = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ArmPracticeIOTalonFX() {
    armPracticeTalon = new TalonFX(Constants.ArmPractice.CAN_ID, new CANBus(Constants.ArmPractice.BUS));
    armPracticeTalon.getConfigurator().apply(Constants.ArmPractice.CONFIG);

    // Get select status signals and set update frequency
    armPracticePosition = armPracticeTalon.getPosition();
    armPracticeVelocity = armPracticeTalon.getVelocity();
    armPracticeAppliedVoltage = armPracticeTalon.getMotorVoltage();
    armPracticeSupplyCurrent = armPracticeTalon.getSupplyCurrent();
    armPracticeTorqueCurrent = armPracticeTalon.getTorqueCurrent();
    armPracticeTempCelsius = armPracticeTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        armPracticePosition,
        armPracticeVelocity,
        armPracticeAppliedVoltage,
        armPracticeSupplyCurrent,
        armPracticeTorqueCurrent,
        armPracticeTempCelsius);

    armPracticeTalon.setPosition(0);
  }

  @Override
  public void updateInputs(ArmPracticeIOInputs inputs) {
    inputs.armPracticeMotorConnected =
        BaseStatusSignal.refreshAll(
                armPracticePosition,
                armPracticeVelocity,
                armPracticeAppliedVoltage,
                armPracticeSupplyCurrent,
                armPracticeTorqueCurrent,
                armPracticeTempCelsius)
            .isOK();
    inputs.armPracticePositionRads =
        Units.rotationsToRadians(armPracticePosition.getValueAsDouble())
            / Constants.ArmPractice.REDUCTION;
    inputs.armPracticeVelocityRadsPerSec =
        Units.rotationsToRadians(armPracticeVelocity.getValueAsDouble())
            / Constants.ArmPractice.REDUCTION;
    inputs.armPracticeAppliedVoltage = armPracticeAppliedVoltage.getValueAsDouble();
    inputs.armPracticeSupplyCurrentAmps = armPracticeSupplyCurrent.getValueAsDouble();
    inputs.armPracticeTorqueCurrentAmps = armPracticeTorqueCurrent.getValueAsDouble();
    inputs.armPracticeTempCelcius = armPracticeTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    armPracticeTalon.setControl(voltageout.withOutput(volts));
  }

  @Override
  public void stop() {
    armPracticeTalon.stopMotor();
  }
}
