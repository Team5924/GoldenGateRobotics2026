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

import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
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
  public enum ShooterRollerState implements VoltageState {
    OFF(() -> 0.0),

    // voltage speed at which to rotate the rollers
    MANUAL(new LoggedTunableNumber("ShooterRoller/Manual", 12.0)),

    // using a double supplier of 0.0 because these will be auto-aim-calculated values
    AUTO_SHOOTING(() -> 0.0),
    NEUTRAL_SHUFFLING(() -> 0.0),
    OPPONENT_SHUFFLING(() -> 0.0),

    // TODO: test and update volts
    BUMPER_SHOOTING(new LoggedTunableNumber("ShooterRoller/BumperShooting", 12.0));

    private final DoubleSupplier voltageSupplier;
  }

  private ShooterRollerState goalState = ShooterRollerState.OFF;

  // Shooter Beam Break
  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  private final boolean isLeft;

  @Setter private double input;

  public ShooterRoller(ShooterRollerIO io, BeamBreakIO beamBreakIO, boolean isLeft) {
    super("ShooterRoller", io);
    this.beamBreakIO = beamBreakIO;
    this.isLeft = isLeft;
  }

  @Override
  protected void handleCurrentVoltageState() {
    switch (goalState) {
      case OFF:
        io.runVolts(0);
        break;

      case MANUAL:
        io.runVolts(getGoalState().getVoltageSupplier().getAsDouble() * input);
        break;

      case AUTO_SHOOTING:
        // TODO: implement
        break;

      case NEUTRAL_SHUFFLING:
        // TODO: implement
        break;

      case OPPONENT_SHUFFLING:
        // TODO: implement
        break;

      case BUMPER_SHOOTING:
        super.handleCurrentVoltageState();
        break;

      default:
        DriverStation.reportWarning(
            "Shooter Roller: " + goalState.name() + " is not a state", null);
        break;
    }
  }

  public void setGoalState(ShooterRollerState goalState) {
    this.goalState = goalState;

    if (isLeft) RobotState.getInstance().setLeftShooterRollerState(goalState);
    else RobotState.getInstance().setRightShooterRollerState(goalState);
  }

  @Override
  public void periodic() {
    super.periodic();
    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("ShooterRoller/BeamBreak", beamBreakInputs);
  }
}
