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
import org.team5924.frc2026.RobotState;
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
    STOW(new LoggedTunableNumber("IntakePivot/StowRads", 2.45)),

    // current at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("IntakePivot/OperatorCurrent", 12.5));

    /** rads are measured from stow position (+ is down) */
    private final DoubleSupplier rads;
  }

  @Getter private IntakePivotState goalState = IntakePivotState.OFF;

  private final Alert intakePivotMotorDisconnected;

  private boolean isAtSetpoint = false;

  protected final Alert overheatAlert;
  protected boolean wasOverheating = false;

  private double lastStateChange = 0.0;
  private double timeSinceLastStateChange = 0.0;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.goalState = IntakePivotState.OFF;
    this.intakePivotMotorDisconnected =
        new Alert("Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);

    overheatAlert = new Alert("intake pivot motor overheating!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    Logger.recordOutput("IntakePivot/GoalState", goalState.toString());
    Logger.recordOutput(
        "IntakePivot/CurrentState", RobotState.getInstance().getIntakePivotState().toString());
    Logger.recordOutput("IntakePivot/TargetRads", goalState.rads.getAsDouble());
    Logger.recordOutput("IntakePivot/CurrentRads", inputs.intakePivotPositionRads);
    Logger.recordOutput("IntakePivot/IsAtSetpoint", isAtSetpoint = isAtSetpoint());
    // Logger.recordOutput(
    //     "IntakePivot/TimeSinceLastStateChange",
    //     timeSinceLastStateChange = RobotState.getTime() - lastStateChange);

    intakePivotMotorDisconnected.set(!inputs.intakePivotMotorConnected);

    handleCurrentState();

    boolean isOverheating = inputs.intakePivotTempCelsius > Constants.OVERHEAT_THRESHOLD;
    overheatAlert.set(isOverheating);
  }

  public boolean isAtSetpoint() {
    // return timeSinceLastStateChange > Constants.IntakePivot.STATE_TIMEOUT
    //     || EqualsUtil.epsilonEquals(
    //       inputs.setpointRads, inputs.intakePivotPositionRads,
    // Constants.IntakePivot.EPSILON_RADS);
    return EqualsUtil.epsilonEquals(
        inputs.setpointRads, inputs.intakePivotPositionRads, Constants.IntakePivot.EPSILON_RADS);
  }

  private void handleCurrentState() {
    switch (RobotState.getInstance().getIntakePivotState()) {
      case MOVING -> {
        if (isAtSetpoint) RobotState.getInstance().setIntakePivotState(goalState);
      }
      case MANUAL -> handleManualState();
      case OFF -> io.stop();
      default -> io.setPosition(goalState.getRads().getAsDouble());
    }
  }

  private void handleManualState() {
    if (!goalState.equals(IntakePivotState.MANUAL)) return;

    if (Math.abs(input) <= Constants.JOYSTICK_DEADZONE) {
      io.runVolts(0);
      return;
    }

    tryRunVolts(IntakePivotState.MANUAL.getRads().getAsDouble() * input);
  }

  public void tryRunVolts(double volts) {
    // if (!(cont = shouldContinueInDirection(volts, inputs.intakePivotPositionRads))) return;

    io.runVolts(volts);
  }

  /**
   * @param rads rads
   * @return -1 for min bound, 0 for within, 1 for upper bound
   */
  public double exceedBounds(double rads) {
    if (rads <= Constants.IntakePivot.MIN_POSITION_RADS) return -1.0;
    if (rads >= Constants.IntakePivot.MAX_POSITION_RADS) return 1.0;
    return 0.0;
  }

  public void setGoalState(IntakePivotState goalState) {
    if (this.goalState.equals(goalState)) return;
    if (goalState.equals(IntakePivotState.MANUAL) && Math.abs(input) <= Constants.JOYSTICK_DEADZONE)
      return;

    this.goalState = goalState;
    switch (goalState) {
      case MANUAL:
        RobotState.getInstance().setIntakePivotState(IntakePivotState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError(
            "IntakePivot: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      case OFF:
        RobotState.getInstance().setIntakePivotState(IntakePivotState.OFF);
        io.stop();
        break;
      default:
        RobotState.getInstance().setIntakePivotState(IntakePivotState.MOVING);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }

    lastStateChange = FieldState.getInstance().getTime();
  }
}
