/*
 * ShooterFlywheel.java
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

package org.team5924.frc2026.subsystems.rollers.shooterFlywheel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class ShooterFlywheel extends SubsystemBase {

  private final ShooterFlywheelIO io;
  private final ShooterFlywheelIOInputsAutoLogged inputs = new ShooterFlywheelIOInputsAutoLogged();

  @Setter private double input;

  @RequiredArgsConstructor
  @Getter
  public enum ShooterFlywheelState {
    OFF(() -> 0.0),
    MOVING(() -> 0.0),

    // current at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("ShooterFlywheel/OperatorCurrent", 200));

    /** measured in rads/sec */
    private final DoubleSupplier velocity;
  }

  @Getter private ShooterFlywheelState goalState = ShooterFlywheelState.OFF;

  private final Alert shooterFlywheelMotorDisconnected;
  private final Notification shooterFlywheelMotorDisconnectedNotification;
  private boolean wasShooterFlywheelMotorConnected = true;
  
  private boolean isAtSetpoint = false;

  protected final Alert overheatAlert;
  protected final Notification overheatNotification;
  protected boolean wasOverheating = false;

  private double lastStateChange = 0.0;
  private double timeSinceLastStateChange = 0.0;

  public ShooterFlywheel(ShooterFlywheelIO io, boolean isLeft) {
    this.io = io;
    this.goalState = ShooterFlywheelState.OFF;
    this.shooterFlywheelMotorDisconnected =
        new Alert("Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);
    this.shooterFlywheelMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Intake Pivot Motor Disconnected", "");

    overheatAlert = new Alert( "intake pivot motor overheating!", Alert.AlertType.kWarning);

    overheatNotification =
        new Notification(
            NotificationLevel.WARNING, "Intake Pivot Overheat Warning", "intake pivot motor overheat imminent!");
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    Logger.processInputs("ShooterFlywheel", inputs);

    Logger.recordOutput("ShooterFlywheel/GoalState", goalState.toString());
    Logger.recordOutput(
        "ShooterFlywheel/CurrentState", RobotState.getInstance().getShooterFlywheelState().toString());
    Logger.recordOutput("ShooterFlywheel/TargetRads", goalState.velocity.getAsDouble());
    Logger.recordOutput("ShooterFlywheel/CurrentRads", inputs.shooterFlywheelPositionRads);
    Logger.recordOutput("ShooterFlywheel/IsAtSetpoint", isAtSetpoint = isAtSetpoint());
    Logger.recordOutput("ShooterFlywheel/TimeSinceLastStateChange", timeSinceLastStateChange = RobotState.getTime() - lastStateChange);

    shooterFlywheelMotorDisconnected.set(!inputs.shooterFlywheelMotorConnected);

    handleCurrentState();

    // prevents error spam
    if (!inputs.shooterFlywheelMotorConnected && wasShooterFlywheelMotorConnected) {
      Elastic.sendNotification(shooterFlywheelMotorDisconnectedNotification);
    }
    wasShooterFlywheelMotorConnected = inputs.shooterFlywheelMotorConnected;

    boolean isOverheating = inputs.shooterFlywheelTempCelsius > Constants.OVERHEAT_THRESHOLD;
    overheatAlert.set(isOverheating);
    if (isOverheating && !wasOverheating) {
      Elastic.sendNotification(overheatNotification);
    }
    wasOverheating = isOverheating;
  }

  public boolean isAtSetpoint() {
    // return timeSinceLastStateChange > Constants.ShooterFlywheel.STATE_TIMEOUT
    //     || EqualsUtil.epsilonEquals(
    //       inputs.setpointRads, inputs.shooterFlywheelPositionRads, Constants.ShooterFlywheel.EPSILON_RADS);
    return EqualsUtil.epsilonEquals(inputs.setpointRads, inputs.shooterFlywheelPositionRads, Constants.ShooterFlywheel.EPSILON_RADS);
  }

  private void handleCurrentState() {
    switch (RobotState.getInstance().getShooterFlywheelState()) {
      case MOVING -> {
        if (isAtSetpoint) RobotState.getInstance().setShooterFlywheelState(goalState);
      }
      case MANUAL -> handleManualState();
      case OFF -> io.stop();
      default -> io.setPosition(goalState.getVelocity().getAsDouble());
    }
  }

  private void handleManualState() {
    if (!goalState.equals(ShooterFlywheelState.MANUAL)) return;

    if (Math.abs(input) <= Constants.JOYSTICK_DEADZONE) {
      io.runVolts(0);
      return;
    }

    tryRunVolts(ShooterFlywheelState.MANUAL.getVelocity().getAsDouble() * input);
  }

  public void tryRunVolts(double volts) {
    // if (!(cont = shouldContinueInDirection(volts, inputs.shooterFlywheelPositionRads))) return;

    io.runVolts(volts);
  }

  /**
   * @param rads rads
   * @return -1 for min bound, 0 for within, 1 for upper bound
   */
  public double exceedBounds(double rads) {
    if (rads <= Constants.ShooterFlywheel.MIN_POSITION_RADS) return -1.0;
    if (rads >= Constants.ShooterFlywheel.MAX_POSITION_RADS) return 1.0;
    return 0.0;
  }

  public void setGoalState(ShooterFlywheelState goalState) {
    if (goalState.equals(ShooterFlywheelState.MANUAL) && Math.abs(input) <= Constants.JOYSTICK_DEADZONE) return;
    this.goalState = goalState;
    switch (goalState) {
      case MANUAL:
        RobotState.getInstance().setShooterFlywheelState(ShooterFlywheelState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError(
            "ShooterFlywheel: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      case OFF:
        RobotState.getInstance().setShooterFlywheelState(ShooterFlywheelState.OFF);
        io.stop();
        break;
      default:
        RobotState.getInstance().setShooterFlywheelState(ShooterFlywheelState.MOVING);
        io.setPosition(goalState.velocity.getAsDouble());
        break;
    }

    lastStateChange = RobotState.getTime();
  }
}