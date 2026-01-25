/*
 * Climb.java
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

package org.team5924.frc2026.subsystems.climb;

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

public class Climb extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public enum ClimbState {
    STOW(new LoggedTunableNumber("Climb/Stow", 0)),
    LEVEL_ONE(new LoggedTunableNumber("Climb/LevelOne", 0)),
    LEVEL_TWO(new LoggedTunableNumber("Climb/LevelTwo", 0)),
    LEVEL_THREE(new LoggedTunableNumber("Climb/LevelThree", 0)),
    CLIMB_DOWN(new LoggedTunableNumber("Climb/ClimbDown", 0)),
    DEPLOY(new LoggedTunableNumber("Climb/Deploy", 0)),
    DROP(new LoggedTunableNumber("Climb/Drop", 0)),
    MOVING(new LoggedTunableNumber("Climb/Moving", 0)),
    // voltage at which the climb subsystem motor moves when controlled by the operator
    OPERATOR_CONTROL(new LoggedTunableNumber("Climb/OperatorVoltage", 4.5));

    private final LoggedTunableNumber rads;

    ClimbState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private ClimbState goalState;

  private final Alert climbMotorDisconnected;
  private final Notification climbMotorDisconnectedNotification;
  private boolean wasClimbMotorConnected = true;

  public Climb(ClimbIO io) {
    this.io = io;
    this.goalState = ClimbState.STOW;
    this.climbMotorDisconnected =
        new Alert("Climb System Motor Disconnected!", Alert.AlertType.kWarning);
    this.climbMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Climb System Motor Disconnected", "");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    Logger.recordOutput("Climb/GoalState", goalState.toString());
    Logger.recordOutput("Climb/CurrentState", RobotState.getInstance().getClimbState());
    Logger.recordOutput("Climb/TargetRads", goalState.rads);

    climbMotorDisconnected.set(!inputs.climbMotorConnected);

    // prevents error spam
    if (!inputs.climbMotorConnected && wasClimbMotorConnected) {
      Elastic.sendNotification(climbMotorDisconnectedNotification);
    }

    wasClimbMotorConnected = inputs.climbMotorConnected;
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setGoalState(ClimbState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case OPERATOR_CONTROL:
        RobotState.getInstance().setClimbState(ClimbState.OPERATOR_CONTROL);
        break;
      case MOVING:
        DriverStation.reportError(
            "Climb: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      default:
        RobotState.getInstance().setClimbState(goalState);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}
