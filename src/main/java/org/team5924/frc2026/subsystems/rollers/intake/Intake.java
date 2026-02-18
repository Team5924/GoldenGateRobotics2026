/*
 * Intake.java
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

package org.team5924.frc2026.subsystems.rollers.intake;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class Intake extends GenericRollerSystem<Intake.IntakeState> {

  @RequiredArgsConstructor
  @Getter
  public enum IntakeState implements VoltageState {
    OFF(() -> 0.0),
    SPITOUT(new LoggedTunableNumber("Intake/SpitOut", -12.0)),
    INTAKE(new LoggedTunableNumber("Intake/Intake", 12.0));

    private final DoubleSupplier voltageSupplier;
  }

  private IntakeState goalState = IntakeState.OFF;

  public Intake(IntakeIO io) {
    super("Intake", io);
  }

  public void setGoalState(IntakeState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setIntakeState(goalState);
  }
}
