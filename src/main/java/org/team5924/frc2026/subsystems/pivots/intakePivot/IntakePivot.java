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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import lombok.Getter;
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
  private double intakePivotPositionSetpointRadiansFromCenter = 0.0;

  @Setter private double input;

  public final SysIdRoutine sysId;

  public enum IntakePivotState {
    OFF(() -> 0.0),
    MOVING(() -> 0.0),

    // voltage at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("IntakePivot/OperatorVoltage", 7.0));

    @Getter private final DoubleSupplier rads;

    IntakePivotState(DoubleSupplier rads) {
      this.rads = rads;
    }
  }

  @Getter private IntakePivotState goalState = IntakePivotState.OFF;

  private final Alert intakePivotMotorDisconnected;
  private final Notification intakePivotMotorDisconnectedNotification;
  private boolean wasIntakePivotMotorConnected = true;

  private Timer stateTimer = new Timer();

  private double bounds;
  private boolean cont;

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

    stateTimer.reset();
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

    // Logger.recordOutput("IntakePivot/idk prob like rads", radsPos);
    // Logger.recordOutput("IntakePivot/input volts", inputVolts);
    Logger.recordOutput("IntakePivot/exceed bounds", bounds);
    Logger.recordOutput("IntakePivot/continue", cont);
    // Logger.recordOutput("IntakePivot/tmp", inputs.intakePivotMotorPosition);
    // Logger.recordOutput("IntakePivot/gcpr", getCurrentPositionRads());

    handleManualState();

    // prevents error spam
    if (!inputs.intakePivotMotorConnected && wasIntakePivotMotorConnected) {
      Elastic.sendNotification(intakePivotMotorDisconnectedNotification);
    }
    wasIntakePivotMotorConnected = inputs.intakePivotMotorConnected;
  }

  public boolean isAtSetpoint() {
    return EqualsUtil.epsilonEquals(
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

  public boolean shouldContinueInDirection(double volts, double rads) {
    double voltDirection = Math.signum(volts);
    return (voltDirection != (bounds = exceedBounds(rads)));
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

    stateTimer.reset();
  }

  // public void setPositionSetpointImpl(double radiansFromCenter, double radPerS) {
  //   Logger.recordOutput("IntakePivot/radiansFromCenter", radiansFromCenter);
  //   io.setPositionSetpoint(radiansFromCenter, radPerS);
  // }

  // private boolean unwrapped(double setpoint) {
  //   // Radians comparison intentional because this is the raw value going into
  //   // rotor.
  //   return (stateTimer.get() - lastStateChangeTime > 0.5)
  //       || EqualsUtil.epsilonEquals(setpoint, getCurrentPositionRads(), Math.toRadians(10.0));
  // }

  // private Command positionSetpointUntilUnwrapped(
  //     DoubleSupplier radiansFromCenter, DoubleSupplier ffVel) {
  //   return run(() -> {
  //         // Intentional do not wrap intakePivot
  //         double setpoint = radiansFromCenter.getAsDouble();
  //         setPositionSetpointImpl(setpoint, unwrapped(setpoint) ? ffVel.getAsDouble() : 0.0);
  //         intakePivotPositionSetpointRadiansFromCenter = setpoint;
  //       })
  //       .until(() -> unwrapped(radiansFromCenter.getAsDouble()));
  // }

  // // FF is in rad/s.
  // public Command positionSetpointCommand(DoubleSupplier radiansFromCenter, DoubleSupplier ffVel)
  // {
  //   return positionSetpointUntilUnwrapped(radiansFromCenter, ffVel)
  //       .andThen(
  //           run(
  //               () -> {
  //                 double setpoint = adjustSetpointForWrap(radiansFromCenter.getAsDouble());
  //                 setPositionSetpointImpl(setpoint, ffVel.getAsDouble());
  //                 intakePivotPositionSetpointRadiansFromCenter = setpoint;
  //               }))
  //       .withName("IntakePivot positionSetpointCommand");
  // }

  // public Command waitForPosition(DoubleSupplier radiansFromCenter, double toleranceRadians) {
  //   return new WaitUntilCommand(
  //           () -> {
  //             return Math.abs(
  //                     new Rotation2d(getCurrentPositionRads())
  //                         .rotateBy(new Rotation2d(radiansFromCenter.getAsDouble()).unaryMinus())
  //                         .getRadians())
  //                 < toleranceRadians;
  //           })
  //       .withName("IntakePivot wait for position");
  // }

  public double getSetpoint() {
    return this.intakePivotPositionSetpointRadiansFromCenter;
  }

  // public void setTeleopDefaultCommand() {
  //   this.setDefaultCommand(
  //       run(() -> {
  //             setPositionSetpointImpl(intakePivotPositionSetpointRadiansFromCenter, 0.0);
  //           })
  //           .withName("IntakePivot Maintain Setpoint (default)"));
  // }
}
