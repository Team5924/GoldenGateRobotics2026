/*
 * ArmPractice.java
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
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ArmPractice extends SubsystemBase {

  private final ArmPracticeIO io;
  private final ArmPracticeIOInputsAutoLogged inputs = new ArmPracticeIOInputsAutoLogged();

  public enum ArmPracticeState {
    STOW(new LoggedTunableNumber("ArmPractice/Stow", Math.toRadians(0))),
    MOVING(new LoggedTunableNumber("ArmPractice/Moving", 0)),
    UP(new LoggedTunableNumber("ArmPractice//Up", Math.toRadians(90))),

    // voltage at which the example subsystem motor moves when controlled by the operator
    OPERATOR_CONTROL(new LoggedTunableNumber("ArmPractice/OperatorVoltage", 4.5));

    private final DoubleSupplier rads;

    ArmPracticeState(DoubleSupplier rads) {
      this.rads = rads;
    }
  }

  @Getter private ArmPracticeState goalState;

  private final Alert armPracticeMotorDisconnected;
  private final Notification armPracticeMotorDisconnectedNotification;
  private boolean wasArmPracticeMotorConnected = true;

  public ArmPractice(ArmPracticeIO io) {
    this.io = io;
    this.goalState = ArmPracticeState.MOVING;
    this.armPracticeMotorDisconnected =
        new Alert("Arm Practice Motor Disconnected!", Alert.AlertType.kWarning);
    this.armPracticeMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Arm Practice Motor Disconnected", "");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ArmPractice", inputs);

    Logger.recordOutput("ArmPractice/GoalState", goalState.toString());
    Logger.recordOutput("ArmPractice/CurrentState", RobotState.getInstance().getArmPracticeState());
    Logger.recordOutput("ArmPractice/TargetRads", goalState.rads);

    armPracticeMotorDisconnected.set(!inputs.armPracticeMotorConnected);

    // prevents error spam
    if (!inputs.armPracticeMotorConnected && wasArmPracticeMotorConnected) {
      Elastic.sendNotification(armPracticeMotorDisconnectedNotification);
    }
    wasArmPracticeMotorConnected = inputs.armPracticeMotorConnected;
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void setGoalState(ArmPracticeState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case OPERATOR_CONTROL:
        RobotState.getInstance().setArmPracticeState(ArmPracticeState.OPERATOR_CONTROL);
        break;
      case MOVING:
        DriverStation.reportError(
            "Arm Practice Subsystem: MOVING is an invalid goal state; it is a transition state",
            null);
        break;
      default:
        RobotState.getInstance().setArmPracticeState(ArmPracticeState.MOVING);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }
  }
}
