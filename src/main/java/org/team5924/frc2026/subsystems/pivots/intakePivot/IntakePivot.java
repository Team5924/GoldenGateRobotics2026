/*
 * IntakePivot.java
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

package org.team5924.frc2026.subsystems.pivots.intakePivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.FieldState;
import org.team5924.frc2026.util.EqualsUtil;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class IntakePivot extends SubsystemBase {

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  @Setter private double input;

  @RequiredArgsConstructor
  @Getter
  public enum IntakePivotState {
    OFF(() -> 0.0),
    MOVING(() -> 0.0),

    DOWN(new LoggedTunableNumber("IntakePivot/DownRads", 0)),
    STOW(new LoggedTunableNumber("IntakePivot/StowRads", 2.05)),
    PHYSICAL_STOW(() -> 2.1),
    SHOOTING(new LoggedTunableNumber("IntakePivot/ShootingRads", 1.5)),

    // current at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("IntakePivot/OperatorCurrent", 12.5));

    /** rads are measured from stow position (+ is down) */
    private final DoubleSupplier rads;
  }

  private final Alert motorDisconnected;
  protected final Alert overheatAlert;

  @Getter private IntakePivotState goalState = IntakePivotState.OFF;
  private IntakePivotState currentState = IntakePivotState.OFF;
  private boolean isAtSetpoint = false;
  private double lastStateChange = 0.0;
  private double timeSinceLastStateChange = 0.0;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.goalState = IntakePivotState.OFF;

    this.motorDisconnected =
        new Alert("Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);
    overheatAlert = new Alert("intake pivot motor overheating!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    motorDisconnected.set(!inputs.motorConnected);
    overheatAlert.set(inputs.tempCelsius > Constants.OVERHEAT_THRESHOLD);

    handleCurrentState();

    Logger.recordOutput("IntakePivot/GoalState", goalState.toString());
    Logger.recordOutput("IntakePivot/CurrentState", currentState.toString());
    Logger.recordOutput("IntakePivot/TargetRads", goalState.rads.getAsDouble());
    Logger.recordOutput("IntakePivot/CurrentRads", inputs.positionRads);
    Logger.recordOutput("IntakePivot/IsAtSetpoint", isAtSetpoint);
    Logger.recordOutput("IntakePivot/TimeSinceLastStateChange", timeSinceLastStateChange);
  }

  public void runCurrent(double amps) {
    io.runCurrent(amps);
  }

  public void setPosition(double rads) {
    io.setPosition(rads);
  }

  public void stop() {
    io.stop();
  }

  public void setGoalState(IntakePivotState goalState) {
    if (this.goalState.equals(goalState)) return;
    if (goalState.equals(IntakePivotState.MANUAL) && Math.abs(input) <= Constants.JOYSTICK_DEADZONE)
      return;

    this.goalState = goalState;
    switch (goalState) {
      case MANUAL -> currentState = IntakePivotState.MANUAL;
      case MOVING ->
          DriverStation.reportError(
              "IntakePivot: MOVING is an invalid goal state; it is a transition state!!", null);
      case OFF -> currentState = IntakePivotState.OFF;
      default -> currentState = IntakePivotState.MOVING;
    }

    lastStateChange = FieldState.getInstance().getTime();
  }

  @SuppressWarnings({"unused"})
  public boolean isAtSetpoint() {
    return (!Constants.IntakePivot.ENABLE_TIMEOUT
            || timeSinceLastStateChange > Constants.IntakePivot.STATE_TIMEOUT)
        && EqualsUtil.epsilonEquals(
            inputs.setpointRads, inputs.positionRads, Constants.IntakePivot.EPSILON_RADS);
  }

  private void handleCurrentState() {
    timeSinceLastStateChange = FieldState.getInstance().getTime() - lastStateChange;
    isAtSetpoint = isAtSetpoint();

    switch (currentState) {
      case MOVING -> {
        setPosition(goalState.getRads().getAsDouble());
        if (isAtSetpoint) currentState = goalState;
      }
      case MANUAL -> handleManualState();
      case OFF -> stop();
      default -> {
        if (!isAtSetpoint) setPosition(goalState.getRads().getAsDouble());
      }
    }
  }

  private void handleManualState() {
    if (!goalState.equals(IntakePivotState.MANUAL)) return;

    if (Math.abs(input) <= Constants.JOYSTICK_DEADZONE) {
      stop();
      return;
    }

    runCurrent(IntakePivotState.MANUAL.getRads().getAsDouble() * input);
  }
}
