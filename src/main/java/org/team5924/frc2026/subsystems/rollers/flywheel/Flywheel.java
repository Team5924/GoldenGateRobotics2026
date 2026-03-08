/*
 * Flywheel.java
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

package org.team5924.frc2026.subsystems.rollers.flywheel;

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

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final boolean isLeft;

  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  @Setter private double input;

  @RequiredArgsConstructor
  @Getter
  public enum FlywheelState {
    OFF(() -> 0.0),
    MOVING(() -> 0.0),
    LAUNCH(() -> 200.0),

    // current at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("Flywheel/OperatorCurrent", 200)),
    AUTO(() -> 0.0),

    B4(() -> 4.0),
    B6(() -> 6.0),
    B8(() -> 8.0),
    B12(() -> 12.0);

    /** measured in rads/sec */
    private final DoubleSupplier velocity;
  }

  @Getter private FlywheelState goalState = FlywheelState.OFF;

  private final Alert flywheelMotorDisconnected;

  private boolean isAtSetpoint = false;

  protected final Alert overheatAlert;

  private final String side;

  @Setter private double autoInput = 0.0;

  // private double lastStateChange = 0.0;
  // private double timeSinceLastStateChange = 0.0;

  public Flywheel(FlywheelIO io, boolean isLeft) {
    side = isLeft ? "Left" : "Right";
    this.isLeft = isLeft;
    this.io = io;
    this.goalState = FlywheelState.OFF;
    this.flywheelMotorDisconnected =
        new Alert(side + " Intake Pivot Motor Disconnected!", Alert.AlertType.kWarning);

    overheatAlert = new Alert(side + " intake pivot motor overheating!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel/" + side, inputs);

    Logger.recordOutput("Flywheel/" + side + "/GoalState", goalState.toString());
    Logger.recordOutput(
        "Flywheel/" + side + "/CurrentState", getRespectiveFlywheelState().toString());
    Logger.recordOutput(
        "Flywheel/" + side + "/CurrentState", getRespectiveFlywheelState().toString());
    Logger.recordOutput("Flywheel/" + side + "/TargetRads", goalState.velocity.getAsDouble());
    Logger.recordOutput("Flywheel/" + side + "/CurrentRads", inputs.flywheelPositionRads);
    Logger.recordOutput("Flywheel/" + side + "/IsAtSetpoint", isAtSetpoint = isAtSetpoint());
    // Logger.recordOutput(
    //     "Flywheel/TimeSinceLastStateChange",
    //     timeSinceLastStateChange = FieldState.getTime() - lastStateChange);

    flywheelMotorDisconnected.set(!inputs.flywheelMotorConnected);

    handleCurrentState();
    // boolean isOverheating = inputs.flywheelTempCelsius > Constants.OVERHEAT_THRESHOLD;
    // overheatAlert.set(isOverheating);
  }

  public boolean isAtSetpoint() {
    // return timeSinceLastStateChange > Constants.Flywheel.STATE_TIMEOUT
    //     || EqualsUtil.epsilonEquals(
    //       inputs.setpointRads, inputs.flywheelPositionRads,
    // Constants.Flywheel.EPSILON_RADS);
    return EqualsUtil.epsilonEquals(
        inputs.setpointVelocity,
        inputs.flywheelVelocityRadsPerSec,
        Constants.GeneralFlywheel.EPSILON_Velocity);
  }

  private void handleCurrentState() {
    switch (getRespectiveFlywheelState()) {
      case MOVING -> {
        if (isAtSetpoint() && goalState != FlywheelState.AUTO)
          setRespectiveFlywheelState(goalState);
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
    if (!goalState.equals(FlywheelState.MANUAL)) return;

    if (Math.abs(input) <= 0.05) {
      io.runVolts(0);
      return;
    }

    io.setVelocity(FlywheelState.MANUAL.getVelocity().getAsDouble() * input);
  }

  public void setGoalState(FlywheelState goalState) {
    if (this.goalState.equals(goalState)) return;

    if (goalState.equals(FlywheelState.MANUAL) && Math.abs(input) <= 0.05) return;

    this.goalState = goalState;
    switch (goalState) {
      case MANUAL:
        setRespectiveFlywheelState(FlywheelState.MANUAL);
        break;
      case MOVING:
        DriverStation.reportError(
            "Flywheel: MOVING is an invalid goal state; it is a transition state!!", null);
        break;
      case OFF:
        setRespectiveFlywheelState(FlywheelState.OFF);
        io.stop();
        break;
      case B4, B6, B8, B12:
        setRespectiveFlywheelState(goalState);
        io.runVolts(goalState.getVelocity().getAsDouble());
        break;
      case AUTO:
        setRespectiveFlywheelState(FlywheelState.MOVING);
        io.setVelocity(autoInput);
        break;
      default:
        setRespectiveFlywheelState(FlywheelState.MOVING);
        io.setVelocity(goalState.velocity.getAsDouble());
        break;
    }

    // lastStateChange = FieldState.getTime();
  }

  private void setRespectiveFlywheelState(FlywheelState state) {
    if (isLeft) RobotState.getInstance().setLeftFlywheelState(state);
    else RobotState.getInstance().setRightFlywheelState(state);
  }

  private FlywheelState getRespectiveFlywheelState() {
    return isLeft
        ? RobotState.getInstance().getLeftFlywheelState()
        : RobotState.getInstance().getRightFlywheelState();
  }
}
