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

package org.team5924.frc2026.subsystems.shooterHood;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ShooterHood extends SubsystemBase {

  private final ShooterHoodIO io;

  public LoggedTunableNumber ShooterHoodPivotTolerance =
      new LoggedTunableNumber("ShooterHoodPivotToleranceRads", .02);
  private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();

  public enum ShooterHoodState { // TODO: update angle rad values
    OFF(new LoggedTunableNumber("ShooterHood/Off", 0)),
    AUTO_SHOOTING(new LoggedTunableNumber("ShooterHood/AutoShooting", Math.toRadians(-1))),
    BUMPER_SHOOTING(new LoggedTunableNumber("ShooterHood/BumperShooting", Math.toRadians(-1))),
    NEUTRAL_SHUFFLING(new LoggedTunableNumber("ShooterHood/NeutralShuffling", Math.toRadians(-1))),
    OPPONENT_SHUFFLING(
        new LoggedTunableNumber("ShooterHood/OpponentShuffling", Math.toRadians(-1))),
    MOVING(new LoggedTunableNumber("ShooterHood/Moving", -1)),
    MANUAL(new LoggedTunableNumber("ShooterHood/Manual", -1));

    private final LoggedTunableNumber rads;

    ShooterHoodState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private ShooterHoodState goalState;

  private final Alert shooterHoodMotorDisconnected;
  private final Notification shooterHoodMotorDisconnectedNotification;
  private boolean wasShooterHoodMotorConnected = true;

  public ShooterHood(ShooterHoodIO io) {
    this.io = io;
    this.goalState = ShooterHoodState.OFF;
    this.shooterHoodMotorDisconnected =
        new Alert("Shooter Hood Motor Disconnected!", Alert.AlertType.kWarning);
    this.shooterHoodMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Shooter Hood Motor Disconnected", "");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterHood", inputs);

    Logger.recordOutput("ShooterHood/GoalState", goalState.toString());
    Logger.recordOutput("ShooterHood/CurrentState", RobotState.getInstance().getShooterHoodState());
    Logger.recordOutput("ShooterHood/TargetRads", goalState.rads.getAsDouble());

    shooterHoodMotorDisconnected.set(!inputs.shooterHoodMotorConnected);

    // prevents error spam
    if (!inputs.shooterHoodMotorConnected && wasShooterHoodMotorConnected) {
      Elastic.sendNotification(shooterHoodMotorDisconnectedNotification);
    }
    wasShooterHoodMotorConnected = inputs.shooterHoodMotorConnected;
  }

  private double getShooterHoodPositionRads() {
    return inputs.shooterHoodPositionRads;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getShooterHoodPositionRads() - this.goalState.rads.getAsDouble())
        < ShooterHoodPivotTolerance.getAsDouble();
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setGoalState(ShooterHoodState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case MANUAL: // TODO: handle manual state
        RobotState.getInstance().setShooterHoodState(ShooterHoodState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError(
            "Shooter Hood: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      default:
        RobotState.getInstance().setShooterHoodState(goalState);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}
