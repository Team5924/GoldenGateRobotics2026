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

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
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

  @Setter private double input;

  public final SysIdRoutine sysId;

  public enum TurretState {
    OFF(() -> 0.0),
    MOVING(() -> 0.0),

    // voltage at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("Turret/OperatorVoltage", 1.0)),

    ZEFO(() -> 0.0);

    @Getter private final DoubleSupplier rads;

    TurretState(DoubleSupplier rads) {
      this.rads = rads;
    }
  }

  @Getter private TurretState goalState = TurretState.OFF;

  private final Alert turretMotorDisconnected;
  private final Notification turretMotorDisconnectedNotification;
  private boolean wasTurretMotorConnected = true;

  private double lastStateChange = 0.0;

  private double exceedBoundsDirection;
  private boolean shouldContinue;

  public Turret(TurretIO io) {
    this.io = io;
    this.goalState = TurretState.OFF;
    this.turretMotorDisconnected =
        new Alert("Turret Motor Disconnected!", Alert.AlertType.kWarning);
    this.turretMotorDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Turret Motor Disconnected", "");

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(.75).per(Seconds),
                Volts.of(1),
                Seconds.of(new LoggedTunableNumber("Turret/SysIdTime", 10.0).getAsDouble()),
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> tryRunVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    Logger.recordOutput("Turret/GoalState", goalState.toString());
    Logger.recordOutput(
        "Turret/CurrentState", RobotState.getInstance().getTurretState().toString());
    Logger.recordOutput("Turret/TargetRads", goalState.rads.getAsDouble());
    Logger.recordOutput("Turret/ExceedBoundsDirection", exceedBoundsDirection);
    Logger.recordOutput("Turret/ShouldContinue", shouldContinue);

    handleCurrentState();

    turretMotorDisconnected.set(!inputs.turretMotorConnected);

    if (!inputs.turretMotorConnected && wasTurretMotorConnected) {
      Elastic.sendNotification(turretMotorDisconnectedNotification);
    }
    wasTurretMotorConnected = inputs.turretMotorConnected;
  }

  public boolean isAtSetpoint() {
    return RobotState.getTime() - lastStateChange < Constants.Turret.STATE_TIMEOUT
        || EqualsUtil.epsilonEquals(
            inputs.setpointRads, inputs.turretPositionRads, Constants.Turret.EPSILON_RADS);
  }

  private void handleCurrentState() {
    switch (RobotState.getInstance().getTurretState()) {
      case MOVING -> {
        if (isAtSetpoint()) RobotState.getInstance().setTurretState(goalState);
      }
      case MANUAL -> handleManualState();
      case OFF -> io.stop();
      default -> io.setPosition(goalState.rads.getAsDouble());
    }
  }

  private void handleManualState() {
    if (!goalState.equals(TurretState.MANUAL)) return;

    if (Math.abs(input) <= Constants.Turret.JOYSTICK_DEADZONE) {
      io.runVolts(0);
      return;
    }

    tryRunVolts(TurretState.MANUAL.getRads().getAsDouble() * input);
  }

  public void tryRunVolts(double volts) {
    // if (!(shouldContinue = shouldContinueInDirection(volts, inputs.turretPositionRads))) return;

    io.runVolts(volts);
  }

  public boolean shouldContinueInDirection(double volts, double rads) {
    double voltDirection = Math.signum(volts);
    return (voltDirection != (exceedBoundsDirection = exceedBoundsDirection(rads)));
  }

  /**
   * @param rads rads
   * @return -1 for min bound, 0 for within, 1 for upper bound
   */
  public double exceedBoundsDirection(double rads) {
    if (rads <= Constants.Turret.MIN_POSITION_RADS) return -1.0;
    if (rads >= Constants.Turret.MAX_POSITION_RADS) return 1.0;
    return 0.0;
  }

  public void setGoalState(TurretState goalState) {
    if (goalState != TurretState.MOVING) this.goalState = goalState;
    switch (goalState) {
      case MANUAL:
        RobotState.getInstance().setTurretState(TurretState.MANUAL);
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

    lastStateChange = RobotState.getTime();
  }

  public double getSetpoint() {
    return this.turretPositionSetpointRadiansFromCenter;
  }
}
