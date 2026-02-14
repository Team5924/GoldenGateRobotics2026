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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {

  private final ArmIO io;

  // private final ArmIOInputsAutoLogged inputs = new ExampleSystemIOInputsAutoLogged();

  public enum ArmState {
    STOW(new LoggedTunableNumber("Arm/Stow", Math.toRadians(0))),
    MOVING(new LoggedTunableNumber("Arm/Moving", 0)),
    UP(new LoggedTunableNumber("Arm/Stow", Math.toRadians(90))),

    // voltage at which the example subsystem motor moves when controlled by the operator
    OPERATOR_CONTROL(new LoggedTunableNumber("Arm/OperatorVoltage", 4.5));

    private final LoggedTunableNumber rads;

    ArmState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private ArmState goalState;

  private final Alert ArmMotorDisconnected;
  private final Notification ArmMotorDisconnectedNotification;
  private boolean wasArmMotorConnected = true;

  public Arm(ArmIO io) {
    this.io = io;
    this.goalState = ArmState.MOVING;
    this.ArmMotorDisconnected =
        new Alert("Arm Motor Disconnected!", Alert.AlertType.kWarning);
    this.ArmMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Arm Motor Disconnected", "");
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("ExampleSystem", inputs);

    Logger.recordOutput("Arm/GoalState", goalState.toString());
    Logger.recordOutput(
        "Arm/CurrentState", RobotState.getInstance().getArmState());
    Logger.recordOutput("Arm/TargetRads", goalState.rads);

    // exampleMotorDisconnected.set(!inputs.exampleMotorConnected);

    // prevents error spam
    // if (!inputs.exampleMotorConnected && wasExampleMotorConnected) {
    Elastic.sendNotification(ArmMotorDisconnectedNotification);
  }

  // wasExampleMotorConnected = inputs.exampleMotorConnected;
  // }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setGoalState(ArmState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case OPERATOR_CONTROL:
        RobotState.getInstance().setArmState(ArmState.OPERATOR_CONTROL);
        break;
      case MOVING:
        DriverStation.reportError(
            "Arm: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      default:
        RobotState.getInstance().setArmState(ArmState.MOVING);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}

