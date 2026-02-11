package org.team5924.frc2026.subsystems.rollers.practiceRoller;

import java.util.function.DoubleSupplier;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class PracticeRoller 
    extends GenericRollerSystem<
        PracticeRoller.PracticeRollerState,
        PracticeRollerIOInputs,
        PracticeRollerIO,
        PracticeRollerIOInputsAutoLogged> {
  @RequiredArgsConstructor
  @Getter
  public enum PracticeRollerState implements VoltageState {
    IDLE(() -> 0.0),
    SHOOTING(new LoggedTunableNumber("PracticeRoller/Shooting", 12.0)),
    INTAKE(new LoggedTunableNumber("ExampleRoller/Intake", -12.0));

    private final DoubleSupplier voltageSupplier;
  }

  private PracticeRollerState goalState = PracticeRollerState.IDLE;

  public PracticeRoller(PracticeRollerIO io) {
    super("PracticeRoller", io, new PracticeRollerIOInputsAutoLogged());
  }

  public void setGoalState(PracticeRollerState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setPracticeRollerState(goalState);
  }
  
}
