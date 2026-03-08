/*
 * ShooterFlywheel.java
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

package org.team5924.frc2026.subsystems.rollers.shooterFlywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.util.EqualsUtil;
import org.team5924.frc2026.util.LaunchCalculator;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class ShooterFlywheel extends SubsystemBase {
  private final ShooterFlywheelIO io;
  private final boolean isLeft;

  private final ShooterFlywheelIOInputsAutoLogged inputs = new ShooterFlywheelIOInputsAutoLogged();

  @Setter private double input;

  @RequiredArgsConstructor
  @Getter
  public enum ShooterFlywheelState {
    OFF(() -> 0.0),
    MOVING(() -> 0.0),
    LAUNCH(() -> 200.0),

    // current at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("ShooterFlywheel/OperatorCurrent", 200)),
    AUTO(() -> 0.0),

    B4(() -> 4.0),
    B6(() -> 6.0),
    B8(() -> 8.0),
    B12(() -> 12.0);

    /** measured in rads/sec */
    private final DoubleSupplier velocity;
  }

  @Getter private ShooterFlywheelState goalState = ShooterFlywheelState.OFF;

  private final Alert shooterFlywheelMotorDisconnected;

  private boolean isAtSetpoint = false;

  protected final Alert overheatAlert;

  private final String side;

  @Setter private double autoInput = 0.0;

  // private double lastStateChange = 0.0;
  // private double timeSinceLastStateChange = 0.0;

  public ShooterFlywheel(ShooterFlywheelIO io, boolean isLeft) {
    side = isLeft ? "Left" : "Right";
    this.isLeft = isLeft;
    this.io = io;
    this.goalState = ShooterFlywheelState.OFF;
    this.shooterFlywheelMotorDisconnected =
        new Alert(side + " Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);

    overheatAlert = new Alert(side + " intake pivot motor overheating!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    Logger.processInputs("ShooterFlywheel/" + side, inputs);

    Logger.recordOutput("ShooterFlywheel/" + side + "/GoalState", goalState.toString());
    Logger.recordOutput(
        "ShooterFlywheel/" + side + "/CurrentState",
        getRespectiveShooterFlywheelState().toString());
    Logger.recordOutput(
        "ShooterFlywheel/" + side + "/TargetRads", goalState.velocity.getAsDouble());
    Logger.recordOutput(
        "ShooterFlywheel/" + side + "/CurrentRads", inputs.shooterFlywheelPositionRads);
    Logger.recordOutput("ShooterFlywheel/" + side + "/IsAtSetpoint", isAtSetpoint = isAtSetpoint());
    // Logger.recordOutput(
    //     "ShooterFlywheel/TimeSinceLastStateChange",
    //     timeSinceLastStateChange = FieldState.getTime() - lastStateChange);

    shooterFlywheelMotorDisconnected.set(!inputs.shooterFlywheelMotorConnected);

    handleCurrentState();
    // boolean isOverheating = inputs.shooterFlywheelTempCelsius > Constants.OVERHEAT_THRESHOLD;
    // overheatAlert.set(isOverheating);
  }

  public boolean isAtSetpoint() {
    // return timeSinceLastStateChange > Constants.ShooterFlywheel.STATE_TIMEOUT
    //     || EqualsUtil.epsilonEquals(
    //       inputs.setpointRads, inputs.shooterFlywheelPositionRads,
    // Constants.ShooterFlywheel.EPSILON_RADS);
    return EqualsUtil.epsilonEquals(
        inputs.setpointVelocity,
        inputs.shooterFlywheelVelocityRadsPerSec,
        Constants.GeneralShooterFlywheel.EPSILON_Velocity);
  }

  private void handleCurrentState() {
    switch (getRespectiveShooterFlywheelState()) {
      case MOVING -> {
        if (isAtSetpoint() && goalState != ShooterFlywheelState.AUTO)
          setRespectiveShooterFlywheelState(goalState);
      }
      case MANUAL -> handleManualState();
      case OFF -> io.stop();
      case LAUNCH -> {
        io.setVelocity(LaunchCalculator.getInstance().getParameters(isLeft).flywheelSpeed());
      }
      case B4, B6, B8, B12 -> {
        io.runVolts(goalState.getVelocity().getAsDouble());
      }
      default -> io.setVelocity(goalState.getVelocity().getAsDouble());
    }
  }

  private void handleManualState() {
    if (!goalState.equals(ShooterFlywheelState.MANUAL)) return;

    if (Math.abs(input) <= 0.05) {
      io.runVolts(0);
      return;
    }

    io.setVelocity(ShooterFlywheelState.MANUAL.getVelocity().getAsDouble() * input);
  }

  public void setGoalState(ShooterFlywheelState goalState) {
    if (this.goalState.equals(goalState)) return;

    if (goalState.equals(ShooterFlywheelState.MANUAL) && Math.abs(input) <= 0.05) return;

    this.goalState = goalState;
    switch (goalState) {
      case MANUAL:
        setRespectiveShooterFlywheelState(ShooterFlywheelState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError(
            "ShooterFlywheel: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      case OFF:
        setRespectiveShooterFlywheelState(ShooterFlywheelState.OFF);
        io.stop();
        break;
      case B4, B6, B8, B12:
        setRespectiveShooterFlywheelState(goalState);
        break;
      case AUTO:
        setRespectiveShooterFlywheelState(ShooterFlywheelState.MOVING);
        io.setVelocity(autoInput);
        break;
      default:
        setRespectiveShooterFlywheelState(ShooterFlywheelState.MOVING);
        io.setVelocity(goalState.velocity.getAsDouble());
        break;
    }

    // lastStateChange = FieldState.getTime();
  }

  private void setRespectiveShooterFlywheelState(ShooterFlywheelState state) {
    if (isLeft) RobotState.getInstance().setLeftShooterFlywheelState(state);
    else RobotState.getInstance().setRightShooterFlywheelState(state);
  }

  private ShooterFlywheelState getRespectiveShooterFlywheelState() {
    return isLeft
        ? RobotState.getInstance().getLeftShooterFlywheelState()
        : RobotState.getInstance().getRightShooterFlywheelState();
  }
}
