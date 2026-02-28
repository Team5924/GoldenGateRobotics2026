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

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.EqualsUtil;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class IntakePivot extends SubsystemBase {

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  @Setter private double input;

  public final SysIdRoutine sysId;

  @RequiredArgsConstructor
  @Getter
  public enum IntakePivotState {
    OFF(() -> 0.0),
    MOVING(() -> 0.0),

    DOWN(new LoggedTunableNumber("IntakePivot/DownRads", 1)), // TODO: tune this, will be higher
    STOW(new LoggedTunableNumber("IntakePivot/StowRads", 0.0)),

    // voltage at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("IntakePivot/OperatorVoltage", 4.0));

    /** rads are measured from stow position (+ is down) */
    private final DoubleSupplier rads;
  }

  @Getter private IntakePivotState goalState = IntakePivotState.OFF;

  private final Alert intakePivotMotorDisconnected;
  private final Notification intakePivotMotorDisconnectedNotification;
  private boolean wasIntakePivotMotorConnected = true;

  private double lastStateChange = 0.0;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.goalState = IntakePivotState.OFF;
    this.intakePivotMotorDisconnected =
        new Alert("Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);
    this.intakePivotMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Intake Pivot Motor Disconnected", "");

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(.75).per(Seconds),
                Volts.of(1),
                Seconds.of(new LoggedTunableNumber("IntakePivot/SysIdTime", 10.0).getAsDouble()),
                (state) -> Logger.recordOutput("IntakePivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> tryRunVolts(voltage.in(Volts)), null, this));
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

    intakePivotMotorDisconnected.set(!inputs.intakePivotMotorConnected);

    handleManualState();

    // prevents error spam
    if (!inputs.intakePivotMotorConnected && wasIntakePivotMotorConnected) {
      Elastic.sendNotification(intakePivotMotorDisconnectedNotification);
    }
    wasIntakePivotMotorConnected = inputs.intakePivotMotorConnected;
  }

  public boolean isAtSetpoint() {
    return RobotState.getTime() - lastStateChange < Constants.IntakePivot.STATE_TIMEOUT
        || EqualsUtil.epsilonEquals(
          inputs.setpointRads, inputs.intakePivotPositionRads, Constants.IntakePivot.EPSILON_RADS);
  }

  private void handleManualState() {
    if (!goalState.equals(IntakePivotState.MANUAL)) return;

    if (Math.abs(input) <= Constants.IntakePivot.JOYSTICK_DEADZONE) {
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

    lastStateChange = RobotState.getTime();
  }
}
