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

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class ExampleRoller extends GenericRollerSystem<ExampleRoller.ExampleRollerState> {
  @RequiredArgsConstructor
  @Getter
  public enum ExampleRollerState implements VoltageState {
    IDLE(new LoggedTunableNumber("ExampleRoller/Idle", 0.0)),
    SHOOTING(new LoggedTunableNumber("ExampleRoller/Shooting", 12.0)),
    INTAKE(new LoggedTunableNumber("ExampleRoller/Intake", -12.0));

    private final DoubleSupplier voltageSupplier;
  }

  private ExampleRollerState goalState = ExampleRollerState.IDLE;

  public ExampleRoller(ExampleRollerIO inputs) {
    super("ExampleRoller", inputs);
  }

  @Override
  public void periodic() {
    ((ExampleRollerIO) io).runVolts(goalState.getVoltageSupplier().getAsDouble());
    super.periodic();
  }

  public void setGoalState(ExampleRollerState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setExampleRollerState(goalState);
  }
}
