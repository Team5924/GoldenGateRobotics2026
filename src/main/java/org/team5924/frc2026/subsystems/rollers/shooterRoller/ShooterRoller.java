/*
 * ShooterRoller.java
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

package org.team5924.frc2026.subsystems.rollers.shooterRoller;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIO;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIOInputsAutoLogged;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class ShooterRoller extends GenericRollerSystem<ShooterRoller.ShooterRollerState> {

  @RequiredArgsConstructor
  @Getter
  public enum ShooterRollerState implements VoltageState { // TODO: update voltage values
    OFF(() -> 0.0),

    // using a double supplier of 0.0 because these will be auto-aim calculated values
    AUTO_SHOOTING(() -> 0.0),
    NEUTRAL_SHUFFLING(() -> 0.0),
    OPPONENT_SHUFFLING(() -> 0.0),

    BUMPER_SHOOTING(new LoggedTunableNumber("ShooterRoller/BumperShooting", 8));

    private final DoubleSupplier voltageSupplier;
  }

  private ShooterRollerState goalState = ShooterRollerState.OFF;

  // Shooter Beam Break
  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  public ShooterRoller(ShooterRollerIO io, BeamBreakIO beamBreakIO) {
    super("ShooterRoller", io);
    this.beamBreakIO = beamBreakIO;
  }

  public void setGoalState(ShooterRollerState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setShooterRollerState(goalState);
  }

  @Override
  public void periodic() {
    super.periodic();
    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("ShooterRoller/BeamBreak", beamBreakInputs);
  }
}
