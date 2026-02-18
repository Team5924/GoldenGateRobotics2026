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

package org.team5924.frc2026.subsystems.intakePivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class IntakePivot extends SubsystemBase {

  private final IntakePivotIO io;

  public static final LoggedTunableNumber PIV_POS_TOLERANCE =
      new LoggedTunableNumber("IntakePivot/PosTolerance", 0.02);
  private final IntakePivotIOInputsAutoLogged pivotInput = new IntakePivotIOInputsAutoLogged();

  // Intake Preset Positions
  public enum IntakePivotState {
    INTAKE_FLOOR(new LoggedTunableNumber("IntakePivot/FloorRads", Math.toRadians(126.0))),
    SCORE_TROUGH(new LoggedTunableNumber("IntakePivot/TroughScoreRads", Math.toRadians(25.639507))),
    STOW(new LoggedTunableNumber("IntakePivot/Stow", Math.toRadians(0))), // TODO: find this
    MOVING(new LoggedTunableNumber("IntakePivot/Moving", 0)),

    // speed at which the intake pivot moves when controlled by the operator (in volts)
    OPERATOR_CONTROL(
        new LoggedTunableNumber("IntakePivot/OperatorRads", 4.5)); // TODO: test and update

    private final LoggedTunableNumber rads;

    IntakePivotState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private IntakePivotState goalState;

  private final Alert intakePivotMotorDisconnected;

  private final Notification intakePivotMotorDisconnectedNotification;

  @Setter private double joystickY;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.goalState = IntakePivotState.MOVING;
    this.intakePivotMotorDisconnected =
        new Alert("Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);
    this.intakePivotMotorDisconnectedNotification =
        new Notification(
            NotificationLevel.WARNING, "Intake Pivot Warning", "Intake Pivot Motor Disconnected");
  }

  @Override
  public void periodic() {
    io.updateInputs(pivotInput);
    Logger.processInputs("IntakePivot", pivotInput);

    Logger.recordOutput("IntakePivot/GoalState", goalState.toString());
    Logger.recordOutput("IntakePivot/CurrentState", RobotState.getInstance().getIntakePivotState());
    Logger.recordOutput("IntakePivot/CurrentRads", getIntakePivotPosRads());
    Logger.recordOutput("IntakePivot/TargetRads", goalState.rads);
    Logger.recordOutput("IntakePivot/IsAtSetpoint", isAtSetpoint());

    intakePivotMotorDisconnected.set(!pivotInput.intakePivotMotorConnected);

    handleOperatorControl();

    /* if (!pivotInput.intakePivotMotorConnected){
        Elastic.sendNotification(intakePivotMotorDisconnectedNotification);
    } */

  }

  private void handleOperatorControl() {
    if (!goalState.equals(IntakePivotState.OPERATOR_CONTROL)) return;
    if (Math.abs(joystickY) <= Constants.IntakePivot.INTAKE_PIVOT_DEADZONE) {
      // RobotState.getInstance().setIntakePivotState(IntakePivotState.MOVING);
      io.setVoltage(0);
      return;
    }

    setVoltage(IntakePivotState.OPERATOR_CONTROL.rads.getAsDouble() * joystickY);
  }

  public double getIntakePivotPosRads() {
    return pivotInput.intakePivotPositionRads;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getIntakePivotPosRads() - this.goalState.rads.getAsDouble())
        < PIV_POS_TOLERANCE.getAsDouble();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setGoalState(IntakePivotState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case OPERATOR_CONTROL:
        RobotState.getInstance().setIntakePivotState(IntakePivotState.OPERATOR_CONTROL);
        break;
      case MOVING:
        DriverStation.reportError("Invalid goal IntakePivotState!", null);
        break;
      default:
        RobotState.getInstance().setIntakePivotState(IntakePivotState.MOVING);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}
