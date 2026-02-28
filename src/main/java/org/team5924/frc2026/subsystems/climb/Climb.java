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
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIO;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIOInputsAutoLogged;
import org.team5924.frc2026.subsystems.turret.Turret.TurretState;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.EqualsUtil;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class Climb extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  @Setter private double input;

  public enum ClimbState { // TODO: Update climb level values - distance not rads!!!
    STOW(new LoggedTunableNumber("Climb/Stow", 0)),
    OFF(() -> 0.0),
    LEVEL_ONE(new LoggedTunableNumber("Climb/LevelOne", 0)),
    LEVEL_TWO(new LoggedTunableNumber("Climb/LevelTwo", 0)),
    LEVEL_THREE(new LoggedTunableNumber("Climb/LevelThree", 0)),
    CLIMB_DOWN(new LoggedTunableNumber("Climb/ClimbDown", 0)),
    DEPLOY(new LoggedTunableNumber("Climb/Deploy", 0)),
    DROP(new LoggedTunableNumber("Climb/Drop", 0)),
    MOVING(() -> 0.0),
    // voltage at which the climb subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("Climb/OperatorVoltage", 4.5));
    
    @Getter private final DoubleSupplier distance;

    ClimbState(DoubleSupplier distance) {
      this.distance = distance;
    }
  }

  @Getter private ClimbState goalState;

  private final Alert climbMotorDisconnected;
  private final Notification climbMotorDisconnectedNotification;
  private boolean wasClimbMotorConnected = true;

  private double lastStateChange = 0.0;

  // Climb Beam Break
  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  public Climb(ClimbIO io, BeamBreakIO beamBreakIO) {
    this.io = io;
    this.goalState = ClimbState.STOW;
    this.climbMotorDisconnected =
        new Alert("Climb System Motor Disconnected!", Alert.AlertType.kWarning);
    this.climbMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Climb System Motor Disconnected", "");
    this.beamBreakIO = beamBreakIO;
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("Climb", inputs);
    Logger.processInputs("Climb/BeamBreak", beamBreakInputs);

    Logger.recordOutput("Climb/GoalState", goalState.toString());
    Logger.recordOutput("Climb/CurrentState", RobotState.getInstance().getClimbState().toString());
    Logger.recordOutput("Climb/TargetRads", distanceToRadians(goalState.distance.getAsDouble()));

    climbMotorDisconnected.set(!inputs.climbMotorConnected);

    handleCurrentState();

    if (!inputs.climbMotorConnected && wasClimbMotorConnected) {
      Elastic.sendNotification(climbMotorDisconnectedNotification);
    }

    wasClimbMotorConnected = inputs.climbMotorConnected;
  }

  public boolean isAtSetpoint() {
    return RobotState.getTime() - lastStateChange > Constants.Climb.STATE_TIMEOUT
        || EqualsUtil.epsilonEquals(
            inputs.setpointRads, inputs.climbPositionRads, Constants.Climb.EPSILON_RADS);
  }

  private void handleCurrentState() {
    switch (RobotState.getInstance().getClimbState()) {
      case MOVING -> {
        if (isAtSetpoint()) RobotState.getInstance().setClimbState(goalState);
      }
      case MANUAL -> handleManualState();
      case OFF -> io.stop();
      default -> io.setPosition(distanceToRadians(goalState.distance.getAsDouble()));
    }
  }

  private void handleManualState() {
    if (!goalState.equals(ClimbState.MANUAL)) return;

    if (Math.abs(input) <= Constants.Climb.JOYSTICK_DEADZONE) {
      io.runVolts(0);
      return;
    }

    io.runVolts(ClimbState.MANUAL.getDistance().getAsDouble() * input);
  }
  
  public void setGoalState(ClimbState goalState) {
    switch (goalState) {
      case MANUAL:
        RobotState.getInstance().setClimbState(ClimbState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError(
            "Climb: MOVING is an invalid goal state; it is a transition state!!", null);
        return;
      default:
        RobotState.getInstance().setClimbState(goalState);
        break;
    }
    this.goalState = goalState;

    lastStateChange = RobotState.getTime();
  }

  private double distanceToRadians(double distance) {
    double r = Constants.Climb.DRUM_CORE_RADIUS_METERS;
    double t = Constants.Climb.ROPE_THICKNESS_METERS;

    double a = t / (4.0 * Math.PI);
    double b = r;
    double c = -distance;

    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) return 0;

    return (-b + Math.sqrt(discriminant)) / (2 * a);
  }
}
