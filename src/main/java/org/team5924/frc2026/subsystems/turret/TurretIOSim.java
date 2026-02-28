/*
 * TurretIOSim.java
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team5924.frc2026.Constants;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
  private double appliedVoltage = 0.0;

  public TurretIOSim(boolean isLeft) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                isLeft
                    ? Constants.TurretLeft.MOTOR_TO_MECHANISM
                    : Constants.TurretRight.MOTOR_TO_MECHANISM,
                isLeft ? Constants.TurretLeft.SIM_MOI : Constants.TurretRight.SIM_MOI),
            gearbox);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (DriverStation.isDisabled()) runVolts(0.0);

    sim.update(Constants.LOOP_PERIODIC_SECONDS);
    inputs.turretMotorConnected = true;
    inputs.turretPositionRads = sim.getAngularPositionRad();
    inputs.turretVelocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.turretAppliedVoltage = appliedVoltage;
    inputs.turretSupplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(double rads) {
    sim.setAngle(rads);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
