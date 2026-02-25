/*
 * ShooterHoodIOSim.java
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

package org.team5924.frc2026.subsystems.pivots.shooterHood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team5924.frc2026.Constants;

public class ShooterHoodIOSim implements ShooterHoodIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
  private double appliedVoltage = 0.0;

  public ShooterHoodIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox, Constants.ShooterHood.SIM_MOI, Constants.ShooterHood.REDUCTION),
            gearbox);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    if (DriverStation.isDisabled()) runVolts(0.0);

    sim.update(Constants.LOOP_PERIODIC_SECONDS);
    inputs.shooterHoodPositionRads = sim.getAngularPositionRad();
    inputs.shooterHoodVelocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.shooterHoodAppliedVoltage = appliedVoltage;
    inputs.shooterHoodSupplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
