/*
 * ModuleIOTalonFXSim.java
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

package org.team5924.frc2026.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.team5924.frc2026.util.PhoenixUtil;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(getDriveTalon()));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(getTurnTalon(), getCancoder()));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update odometry inputs
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();

    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }
}
