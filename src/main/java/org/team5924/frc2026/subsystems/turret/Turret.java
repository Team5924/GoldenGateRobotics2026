/*
 * Turret.java
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

package org.team5924.frc2026.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;
import org.team5924.frc2026.util.EqualsUtil;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double turretPositionSetpointRadiansFromCenter = 0.0;

  public enum TurretState {
    OFF(new LoggedTunableNumber("Turret/Off", Math.toRadians(0))),
    MOVING(new LoggedTunableNumber("Turret/Moving", 0)),

    // voltage at which the example subsystem motor moves when controlled by the operator
    OPERATOR_CONTROL(new LoggedTunableNumber("Turret/OperatorVoltage", 2));

    private final LoggedTunableNumber rads;

    TurretState(LoggedTunableNumber rads) {
      this.rads = rads;
    }
  }

  @Getter private TurretState goalState;

  private final Alert turretMotorDisconnected;
  private final Notification turretMotorDisconnectedNotification;
  private boolean wasTurretMotorConnected = true;

  private Timer stateTimer = new Timer();
  private double lastStateChangeTime;

  public Turret(TurretIO io) {
    this.io = io;
    this.goalState = TurretState.OFF;
    this.turretMotorDisconnected =
        new Alert("Turret Motor Disconnected!", Alert.AlertType.kWarning);
    this.turretMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Turret Motor Disconnected", "");

    stateTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TurretSystem", inputs);

    Logger.recordOutput("Turret/GoalState", goalState.toString());
    Logger.recordOutput("Turret/CurrentState", RobotState.getInstance().getTurretState());
    Logger.recordOutput("Turret/TargetRads", goalState.rads.getAsDouble());

    turretMotorDisconnected.set(!inputs.turretMotorConnected);

    // prevents error spam
    if (!inputs.turretMotorConnected && wasTurretMotorConnected) {
      Elastic.sendNotification(turretMotorDisconnectedNotification);
    }
    wasTurretMotorConnected = inputs.turretMotorConnected;
  }

  public void runVolts(double volts) {
    if (!continueInDirection(getCurrentPositionRads(), volts)) return;

    io.runVolts(volts);
  }

  public boolean continueInDirection(double rads, double volts) {
    double currentRads = getCurrentPositionRads();
    // if (currentRads )
    return false;
  }

  public void setGoalState(TurretState goalState) {
    this.goalState = goalState;
    switch (goalState) {
      case OPERATOR_CONTROL:
        RobotState.getInstance().setTurretState(TurretState.OPERATOR_CONTROL);
        // turretPositionSetpointRadiansFromCenter =
        break;
      case MOVING:
        DriverStation.reportError(
            "Turret: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      case OFF:
        RobotState.getInstance().setTurretState(TurretState.OFF);
        io.stop();
        break;
      default:
        RobotState.getInstance().setTurretState(TurretState.MOVING);
        io.setPosition(goalState.rads.getAsDouble());
        break;
    }

    lastStateChangeTime = stateTimer.get();
  }

  public void setPositionSetpointImpl(double radiansFromCenter, double radPerS) {
    Logger.recordOutput("Turret/radiansFromCenter", radiansFromCenter);
    io.setPositionSetpoint(radiansFromCenter, radPerS);
  }

  private double adjustSetpointForWrap(double radiansFromCenter) {
    // We have two options the raw radiansFromCenter or +/- 2 * PI.
    double alternative = radiansFromCenter - 2.0 * Math.PI;
    if (radiansFromCenter < 0.0) {
      alternative = radiansFromCenter + 2.0 * Math.PI;
    }
    if (Math.abs(getCurrentPositionRads() - alternative)
        < Math.abs(getCurrentPositionRads() - radiansFromCenter)) {
      return alternative;
    }
    return radiansFromCenter;
  }

  private boolean unwrapped(double setpoint) {
    // Radians comparison intentional because this is the raw value going into
    // rotor.
    return (stateTimer.get() - lastStateChangeTime > 0.5)
        || EqualsUtil.epsilonEquals(setpoint, getCurrentPositionRads(), Math.toRadians(10.0));
  }

  private Command positionSetpointUntilUnwrapped(
      DoubleSupplier radiansFromCenter, DoubleSupplier ffVel) {
    return run(() -> {
          // Intentional do not wrap turret
          double setpoint = radiansFromCenter.getAsDouble();
          setPositionSetpointImpl(setpoint, unwrapped(setpoint) ? ffVel.getAsDouble() : 0.0);
          turretPositionSetpointRadiansFromCenter = setpoint;
        })
        .until(() -> unwrapped(radiansFromCenter.getAsDouble()));
  }

  // FF is in rad/s.
  public Command positionSetpointCommand(DoubleSupplier radiansFromCenter, DoubleSupplier ffVel) {
    return positionSetpointUntilUnwrapped(radiansFromCenter, ffVel)
        .andThen(
            run(
                () -> {
                  double setpoint = adjustSetpointForWrap(radiansFromCenter.getAsDouble());
                  setPositionSetpointImpl(setpoint, ffVel.getAsDouble());
                  turretPositionSetpointRadiansFromCenter = setpoint;
                }))
        .withName("Turret positionSetpointCommand");
  }

  public Command waitForPosition(DoubleSupplier radiansFromCenter, double toleranceRadians) {
    return new WaitUntilCommand(
            () -> {
              return Math.abs(
                      new Rotation2d(getCurrentPositionRads())
                          .rotateBy(new Rotation2d(radiansFromCenter.getAsDouble()).unaryMinus())
                          .getRadians())
                  < toleranceRadians;
            })
        .withName("Turret wait for position");
  }

  public double getSetpoint() {
    return this.turretPositionSetpointRadiansFromCenter;
  }

  public double getCurrentPositionRads() {
    return Units.rotationsToRadians(inputs.cancoderAbsolutePosition);
  }

  public void setTeleopDefaultCommand() {
    this.setDefaultCommand(
        run(() -> {
              setPositionSetpointImpl(turretPositionSetpointRadiansFromCenter, 0.0);
            })
            .withName("Turret Maintain Setpoint (default)"));
  }
}
