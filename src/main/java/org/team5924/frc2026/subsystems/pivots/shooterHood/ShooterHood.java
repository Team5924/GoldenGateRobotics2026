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

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
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
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ShooterHood extends SubsystemBase {
  private final ShooterHoodIO io;
  private final boolean isLeft;

  public LoggedTunableNumber tolerance =
      new LoggedTunableNumber("ShooterHood/ToleranceDeg", Constants.GeneralShooterHood.EPSILON_RADS);
  private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();

  @RequiredArgsConstructor
  @Getter
  public enum ShooterHoodState {
    OFF(() -> 0.0),

    // voltage speed at which to rotate the hood
    MANUAL((new LoggedTunableNumber("ShooterHood/Manual", 1))),

    // using a double supplier of 0.0 because these will be auto-aim-calculated values
    AUTO_SHOOTING(() -> 0.0),
    NEUTRAL_SHUFFLING(() -> 0.0),
    OPPONENT_SHUFFLING(() -> 0.0),

    // TODO: test and update angle (rads)
    BUMPER_SHOOTING(new LoggedTunableNumber("ShooterHood/BumperShooting", Math.toRadians(82))),

    // in-between state
    MOVING(() -> 0.0);

    private final DoubleSupplier rads;
  }

  @Getter private ShooterHoodState goalState;

  @Setter private double input;

  public final SysIdRoutine sysId;

  private final Alert shooterHoodMotorDisconnected;
  private final Notification shooterHoodMotorDisconnectedNotification;
  private boolean wasShooterHoodMotorConnected = true;

  private final Alert notImplementedAlert;
  private boolean showNotImplementedAlert;

  public ShooterHood(ShooterHoodIO io, boolean isLeft) {
    this.io = io;
    this.isLeft = isLeft;

    goalState = ShooterHoodState.OFF;
    shooterHoodMotorDisconnected =
        new Alert("Shooter Hood Motor Disconnected!", Alert.AlertType.kWarning);
    shooterHoodMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Shooter Hood Motor Disconnected", "");

    notImplementedAlert = new Alert("Auto Shooting not yet implemented!", Alert.AlertType.kWarning);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(.75).per(Seconds),
                Volts.of(1),
                Seconds.of(new LoggedTunableNumber("ShooterHood/SysIdTime", 10.0).getAsDouble()),
                (state) -> Logger.recordOutput("ShooterHood/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterHood", inputs);

    Logger.recordOutput("ShooterHood/GoalState", goalState.toString());
    Logger.recordOutput("ShooterHood/CurrentState", getRespectiveShooterHoodState().toString());
    Logger.recordOutput("ShooterHood/TargetRads", goalState.rads.getAsDouble());

    shooterHoodMotorDisconnected.set(!inputs.shooterHoodMotorConnected);

    handleCurrentState();

    // prevents error spam
    if (!inputs.shooterHoodMotorConnected && wasShooterHoodMotorConnected) {
      Elastic.sendNotification(shooterHoodMotorDisconnectedNotification);
    }
    wasShooterHoodMotorConnected = inputs.shooterHoodMotorConnected;
  }

  private void handleCurrentState() {
    showNotImplementedAlert = false;
    switch (getRespectiveShooterHoodState()) {
      case MOVING -> {
        if (isAtSetpoint()) setRespectiveShooterHoodState(goalState);
      }
      case AUTO_SHOOTING, NEUTRAL_SHUFFLING, OPPONENT_SHUFFLING -> {
        showNotImplementedAlert = true; // TODO: handle this sometime
      }
      case MANUAL -> handleManualState();
      case OFF -> io.stop();
      default -> io.setPosition(goalState.rads.getAsDouble());
    }

    notImplementedAlert.set(showNotImplementedAlert);
  }

  private void handleManualState() {
    if (!goalState.equals(ShooterHoodState.MANUAL)) return;

    if (Math.abs(input) <= Constants.GeneralShooterHood.JOYSTICK_DEADZONE) {
      io.runVolts(0);
      return;
    }

    io.runVolts(ShooterHoodState.MANUAL.getRads().getAsDouble() * input);
  }

  private double getShooterHoodPositionRads() {
    return inputs.shooterHoodPositionRads;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getShooterHoodPositionRads() - this.goalState.rads.getAsDouble())
        < Units.degreesToRadians(tolerance.getAsDouble());
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setPosition(double rads) {
    io.setPosition(rads);
  }

  public void setGoalState(ShooterHoodState goalState) {
    if (this.goalState.equals(goalState)) return;

    this.goalState = goalState;
    switch (goalState) {
      case MANUAL, AUTO_SHOOTING, NEUTRAL_SHUFFLING, OPPONENT_SHUFFLING:
        setRespectiveShooterHoodState(goalState);
        break;
      case MOVING:
        DriverStation.reportError(
            "Shooter Hood: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      default:
        setRespectiveShooterHoodState(goalState);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
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
