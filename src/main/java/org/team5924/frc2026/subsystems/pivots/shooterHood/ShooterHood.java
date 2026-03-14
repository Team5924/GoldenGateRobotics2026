/*
 * ShooterHood.java
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

package org.team5924.frc2026.subsystems.pivots.shooterHood;

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

public class ShooterHood extends SubsystemBase {
  private final ShooterHoodIO io;
  private final boolean isLeft;

  private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();

  @RequiredArgsConstructor
  @Getter
  public enum ShooterHoodState {
    OFF(() -> 0.0),
    ZERO(() -> 0.0),

    // voltage speed at which to rotate the hood
    MANUAL((new LoggedTunableNumber("ShooterHood/Manual", 1))),

    // using a double supplier of 0.0 because these will be auto-aim-calculated values
    AUTO_SHOOTING(() -> 0.0),
    NEUTRAL_SHUFFLING(() -> 0.0),
    OPPONENT_SHUFFLING(() -> 0.0),

    // TODO: test and update angle (rads)
    BUMPER_SHOOTING(new LoggedTunableNumber("ShooterHood/BumperShooting", Math.toRadians(30))),
    AUTO(() -> 0.0),

    // in-between state
    MOVING(() -> 0.0);

    private final DoubleSupplier rads;
  }

  private final String side;

  private final Alert motorDisconnectedAlert;
  protected final Alert overheatAlert;
  private final Alert notImplementedAlert;
  private boolean showNotImplementedAlert;

  @Getter private ShooterHoodState goalState;
  private boolean isAtSetpoint = false;
  private double lastStateChange = 0.0;
  private double timeSinceLastStateChange = 0.0;

  @Setter private double input;
  @Setter private double autoInput = 0.0;

  public ShooterHood(ShooterHoodIO io, boolean isLeft) {
    side = isLeft ? "Left" : "Right";
    this.io = io;
    this.isLeft = isLeft;

    goalState = ShooterHoodState.OFF;
    motorDisconnectedAlert =
        new Alert(side + " Shooter Hood Motor Disconnected!", Alert.AlertType.kWarning);
    notImplementedAlert =
        new Alert(side + " Auto Shooting not yet implemented!", Alert.AlertType.kWarning);
    overheatAlert = new Alert(side + " Shooter Hood motor overheating!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterHood/" + side, inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);
    overheatAlert.set(inputs.tempCelsius > Constants.OVERHEAT_THRESHOLD);

    handleCurrentState();
    Logger.recordOutput("ShooterHood/" + side + "/GoalState", goalState.toString());
    Logger.recordOutput(
        "ShooterHood/" + side + "CurrentState", getRespectiveShooterHoodState().toString());
    Logger.recordOutput("ShooterHood/" + side + "/TargetRads", getTargetRads());
    Logger.recordOutput("ShooterHood/" + side + "/CurrentRads", inputs.positionRads);
    Logger.recordOutput("ShooterHood/" + side + "/IsAtSetpoint", isAtSetpoint);
    Logger.recordOutput(
        "ShooterHood/" + side + "/TimeSinceLastStateChange", timeSinceLastStateChange);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setPosition(double rads) {
    io.setPosition(rads);
  }

  public void stop() {
    io.stop();
  }

  public void setGoalState(ShooterHoodState goalState) {
    if (this.goalState.equals(goalState)) return;
    if (goalState.equals(ShooterHoodState.MANUAL) && Math.abs(input) <= Constants.JOYSTICK_DEADZONE)
      return;

    this.goalState = goalState;
    switch (goalState) {
      case MANUAL, AUTO_SHOOTING, NEUTRAL_SHUFFLING, OPPONENT_SHUFFLING ->
          setRespectiveShooterHoodState(goalState);
      case MOVING ->
          DriverStation.reportError(
              side + " Shooter Hood: MOVING is an invalid goal state; it is a transition state!!",
              null);
      case AUTO -> setRespectiveShooterHoodState(ShooterHoodState.MOVING);
      default -> setRespectiveShooterHoodState(goalState);
    }

    lastStateChange = FieldState.getInstance().getTime();
  }

  @SuppressWarnings("unused")
  public boolean isAtSetpoint() {
    return (!Constants.GeneralShooterHood.ENABLE_TIMEOUT
            || timeSinceLastStateChange > Constants.GeneralShooterHood.STATE_TIMEOUT)
        && EqualsUtil.epsilonEquals(
            inputs.positionRads, getTargetRads(), Constants.GeneralShooterHood.EPSILON_RADS);
  }

  private double getTargetRads() {
    return goalState == ShooterHoodState.AUTO ? autoInput : goalState.rads.getAsDouble();
  }

  private void handleCurrentState() {
    timeSinceLastStateChange = FieldState.getInstance().getTime() - lastStateChange;
    isAtSetpoint = isAtSetpoint();

    showNotImplementedAlert = false;
    switch (getRespectiveShooterHoodState()) {
      case MOVING -> {
        setPosition(goalState.getRads().getAsDouble());
        if (isAtSetpoint() && goalState != ShooterHoodState.AUTO)
          setRespectiveShooterHoodState(goalState);
      }
      case AUTO_SHOOTING, NEUTRAL_SHUFFLING, OPPONENT_SHUFFLING -> {
        showNotImplementedAlert = true; // TODO: handle this sometime
      }
      case MANUAL -> handleManualState();
      case OFF -> stop();
      default -> {
        if (!isAtSetpoint) setPosition(goalState.getRads().getAsDouble());
      }
    }

    notImplementedAlert.set(showNotImplementedAlert);
  }

  private void handleManualState() {
    if (!goalState.equals(ShooterHoodState.MANUAL)) return;

    if (Math.abs(input) <= Constants.JOYSTICK_DEADZONE) {
      stop();
      return;
    }

    runVolts(ShooterHoodState.MANUAL.getRads().getAsDouble() * input);
  }

  private void setRespectiveShooterHoodState(ShooterHoodState state) {
    if (isLeft) RobotState.getInstance().setLeftShooterHoodState(state);
    else RobotState.getInstance().setRightShooterHoodState(state);
  }

  private ShooterHoodState getRespectiveShooterHoodState() {
    return isLeft
        ? RobotState.getInstance().getLeftShooterHoodState()
        : RobotState.getInstance().getRightShooterHoodState();
  }
}
