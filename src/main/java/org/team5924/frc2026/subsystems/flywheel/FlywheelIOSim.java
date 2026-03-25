/*
 * FlywheelIOSim.java
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

package org.team5924.frc2026.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team5924.frc2026.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
  private double appliedVoltage = 0.0;
  private double setpoint;

  public FlywheelIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox, Constants.Flywheel.SIM_MOI, Constants.Flywheel.MOTOR_TO_MECHANISM),
            gearbox);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (DriverStation.isDisabled()) stop();

    sim.update(Constants.LOOP_PERIODIC_SECONDS);
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRotationsPerSec = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.setpointVelocityRotationsPerSec = setpoint;

    for (int i = 0; i < 4; ++i) {
      inputs.motorConnected[i] = true;
      inputs.tempCelsius[i] = 25.0;
    }
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setVelocity(double velocity) {
    setpoint = velocity;
    sim.setAngularVelocity(Units.rotationsToRadians(setpoint));
  }

  @Override
  public void stop() {
    setpoint = 0.0;
    runVolts(0.0);
  }
}
