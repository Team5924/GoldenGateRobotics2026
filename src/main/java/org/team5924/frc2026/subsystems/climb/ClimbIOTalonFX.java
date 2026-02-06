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
import org.team5924.frc2026.Constants;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX climbTalon;
  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVoltage;
  private final StatusSignal<Current> climbSupplyCurrent;
  private final StatusSignal<Current> climbTorqueCurrent;
  private final StatusSignal<Temperature> climbTempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ClimbIOTalonFX() {
    climbTalon = new TalonFX(Constants.Climb.CAN_ID, new CANBus(Constants.Climb.BUS));
    climbTalon.getConfigurator().apply(Constants.Climb.CONFIG);

    // Get select status signals and set update frequency
    climbPosition = climbTalon.getPosition();
    climbVelocity = climbTalon.getVelocity();
    climbAppliedVoltage = climbTalon.getMotorVoltage();
    climbSupplyCurrent = climbTalon.getSupplyCurrent();
    climbTorqueCurrent = climbTalon.getTorqueCurrent();
    climbTempCelsius = climbTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        climbPosition,
        climbVelocity,
        climbAppliedVoltage,
        climbSupplyCurrent,
        climbTorqueCurrent,
        climbTempCelsius);

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
    inputs.climbPositionRads =
        Units.rotationsToRadians(climbPosition.getValueAsDouble()) / Constants.Climb.REDUCTION;
    inputs.climbVelocityRadsPerSec =
        Units.rotationsToRadians(climbVelocity.getValueAsDouble()) / Constants.Climb.REDUCTION;
    inputs.climbAppliedVoltage = climbAppliedVoltage.getValueAsDouble();
    inputs.climbSupplyCurrentAmps = climbSupplyCurrent.getValueAsDouble();
    inputs.climbTorqueCurrentAmps = climbTorqueCurrent.getValueAsDouble();
    inputs.climbTempCelsius = climbTempCelsius.getValueAsDouble();
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
    climbTalon.setControl(
        positionOut.withPosition(rads * Constants.Climb.REDUCTION * Units.radiansToRotations(1.0)));
  }
}
