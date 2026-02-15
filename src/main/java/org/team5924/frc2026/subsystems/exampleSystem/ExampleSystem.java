/*
 * ExampleSystem.java
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

package org.team5924.frc2026.subsystems.exampleSystem;

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

public class ExampleSystem extends SubsystemBase {

  private final ExampleSystemIO io;
  private final ExampleSystemIOInputsAutoLogged inputs = new ExampleSystemIOInputsAutoLogged();

  public enum ExampleSystemState {
    STOW(new LoggedTunableNumber("ExampleSystem/Stow", Math.toRadians(0))),
    MOVING(new LoggedTunableNumber("ExampleSystem/Moving", 0)),
    UP(new LoggedTunableNumber("ExampleSystem/Up", Math.toRadians(90))),

    // voltage at which the example subsystem motor moves when controlled by the operator
    OPERATOR_CONTROL(new LoggedTunableNumber("ExampleSystem/OperatorVoltage", 4.5));

    private final LoggedTunableNumber rads;

    ExampleSystemState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private ExampleSystemState goalState;

  private final Alert exampleMotorDisconnected;
  private final Notification exampleMotorDisconnectedNotification;
  private boolean wasExampleMotorConnected = true;

  public ExampleSystem(ExampleSystemIO io) {
    this.io = io;
    this.goalState = ExampleSystemState.STOW;
    this.exampleMotorDisconnected =
        new Alert("Example System Motor Disconnected!", Alert.AlertType.kWarning);
    this.exampleMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Example System Motor Disconnected", "");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ExampleSystem", inputs);

    Logger.recordOutput("ExampleSystem/GoalState", goalState.toString());
    Logger.recordOutput(
        "ExampleSystem/CurrentState", RobotState.getInstance().getExampleSystemState().toString());
    Logger.recordOutput("ExampleSystem/TargetRads", goalState.rads.getAsDouble());

    exampleMotorDisconnected.set(!inputs.exampleMotorConnected);

    // prevents error spam
    if (!inputs.exampleMotorConnected && wasExampleMotorConnected) {
      Elastic.sendNotification(exampleMotorDisconnectedNotification);
    }
    wasExampleMotorConnected = inputs.exampleMotorConnected;
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setGoalState(ExampleSystemState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case OPERATOR_CONTROL:
        RobotState.getInstance().setExampleSystemState(ExampleSystemState.OPERATOR_CONTROL);
        break;
      case MOVING:
        DriverStation.reportError(
            "Example Subsystem: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      default:
        RobotState.getInstance().setExampleSystemState(ExampleSystemState.MOVING);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}
