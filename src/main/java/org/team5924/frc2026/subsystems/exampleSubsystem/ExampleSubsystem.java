/*
 * ExampleSubsystem.java
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

package org.team5924.frc2026.subsystems.exampleSubsystem;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ExampleSubsystem extends SubsystemBase {

    private final ExampleSubsystemIO io;
    private final ExampleSubsystemIOInputsAutoLogged inputs =
            new ExampleSubsystemIOInputsAutoLogged();

    public enum ExampleSubsystemState {
        STOW(new LoggedTunableNumber("ExampleSubsystem/Stow", Math.toRadians(0))),
        MOVING(new LoggedTunableNumber("ExampleSubsystem/Moving", 0)),
        UP(new LoggedTunableNumber("ExampleSubsystem/Stow", Math.toRadians(90))),

        // voltage at which the intake pivot moves when controlled by the operator
        OPERATOR_CONTROL(new LoggedTunableNumber("ExampleSubsystem/OperatorVoltage", 4.5));

        private final LoggedTunableNumber rads;

        ExampleSubsystemState(LoggedTunableNumber rads) {
            this.rads = rads;
        }
    }

    @Getter private ExampleSubsystemState goalState;

    private final Alert exampleSubsystemMotorDisconnected;
    private final Notification exampleSubsystemMotorDisconnectedNotification;

    public ExampleSubsystem(ExampleSubsystemIO io) {
        this.io = io;
        this.goalState = ExampleSubsystemState.MOVING;
        this.exampleSubsystemMotorDisconnected =
                new Alert("Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);
        this.exampleSubsystemMotorDisconnectedNotification =
                new Notification(
                        NotificationLevel.WARNING, "Example Subsystem Motor Disconnected", "");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);

        Logger.recordOutput("IntakePivot/GoalState", goalState.toString());
        Logger.recordOutput(
                "IntakePivot/CurrentState", RobotState.getInstance().getExampleSubsystemState());
        Logger.recordOutput("IntakePivot/TargetRads", goalState.rads);

        exampleSubsystemMotorDisconnected.set(!inputs.exampleSubsystemMotorConnected);

        /* if (!pivotInput.exampleSubsystemConnected){
            Elastic.sendNotification(exampleSubsystemDisconnectedNotification);
        } */
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setGoalState(ExampleSubsystemState goalState) {
        this.goalState = goalState;
        switch (goalState) {
            case OPERATOR_CONTROL:
                RobotState.getInstance()
                        .setExampleSubsystemState(ExampleSubsystemState.OPERATOR_CONTROL);
                break;
            case MOVING:
                DriverStation.reportError(
                        "Example Subsystem: MOVING is an invalid goal state; it is a transition state!!",
                        null);
                break;
            default:
                RobotState.getInstance().setExampleSubsystemState(ExampleSubsystemState.MOVING);
                io.setPosition(goalState.rads.getAsDouble());
                break;
        }
    }
}
