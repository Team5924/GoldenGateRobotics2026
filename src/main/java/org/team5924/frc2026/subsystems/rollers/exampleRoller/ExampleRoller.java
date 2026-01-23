/*
 * ExampleRoller.java
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

package org.team5924.frc2026.subsystems.rollers.exampleRoller;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class ExampleRoller
    extends GenericRollerSystem<
        ExampleRoller.ExampleRollerState, ExampleRollerIOInputs, ExampleRollerIO> {
  @RequiredArgsConstructor
  @Getter
  public enum ExampleRollerState implements VoltageState {
    IDLE(new LoggedTunableNumber("ExampleRoller/Idle", 0.0)),
    SHOOTING(new LoggedTunableNumber("ExampleRoller/Shooting", 12.0)),
    INTAKE(new LoggedTunableNumber("ExampleRoller/Intake", -12.0));

    private final LoggedTunableNumber voltageSupplier;
  }

  private ExampleRollerState goalState = ExampleRollerState.IDLE;

  private final ExampleRollerIOInputsAutoLogged inputs = new ExampleRollerIOInputsAutoLogged();

  public ExampleRoller(ExampleRollerIO io) {
    super("ExampleRoller", io);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.motorConnected);

    super.periodic();

    if (!inputs.motorConnected && wasMotorConnected) {
      Elastic.sendNotification(disconnectedNotification);
    }
    wasMotorConnected = inputs.motorConnected;
  }

  public void setGoalState(ExampleRollerState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setExampleRollerState(goalState);
  }
}
