/*
 * Arm.java
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

package org.team5924.frc2026.subsystems.arm;

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

public class Arm extends SubsystemBase {

  private final ArmIO io;

  public LoggedTunableNumber ArmPivotTolerance =
      new LoggedTunableNumber("ArmPivotToleranceRads", .02);
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public enum ArmState { // TODO: update angle rad values
    OFF(new LoggedTunableNumber("Arm/Off", 0)),
    MOVING(new LoggedTunableNumber("Arm/Moving", -1)),
    MANUAL(new LoggedTunableNumber("Arm/Manual", -1));

    private final LoggedTunableNumber rads;

    ArmState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private ArmState goalState;

  private final Alert armMotorDisconnected;
  private final Notification armMotorDisconnectedNotification;
  private boolean wasArmMotorConnected = true;

  public Arm(ArmIO io) {
    this.io = io;
    this.goalState = ArmState.OFF;
    this.armMotorDisconnected = new Alert("Arm Motor Disconnected!", Alert.AlertType.kWarning);
    this.armMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Arm Motor Disconnected", "");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    Logger.recordOutput("Arm", goalState.toString());
    Logger.recordOutput("Arm", RobotState.getInstance().getArmState());
    Logger.recordOutput("Arm", goalState.rads.getAsDouble());

    armMotorDisconnected.set(!inputs.armMotorConnected);

    // prevents error spam
    if (!inputs.armMotorConnected && wasArmMotorConnected) {
      Elastic.sendNotification(armMotorDisconnectedNotification);
    }
    wasArmMotorConnected = inputs.armMotorConnected;
  }

  private double getArmPositionRads() {
    return inputs.armPositionRads;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getArmPositionRads() - this.goalState.rads.getAsDouble())
        < ArmPivotTolerance.getAsDouble();
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setGoalState(ArmState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case MANUAL: // TODO: handle manual state
        RobotState.getInstance().setArmState(ArmState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError(
            "Arm: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      default:
        RobotState.getInstance().setArmState(goalState);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}
