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

package org.team5924.frc2026.subsystems.flywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.util.EqualsUtil;
import org.team5924.frc2026.util.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;

  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  @Setter private double input;

  @RequiredArgsConstructor
  @Getter
  public enum FlywheelState {
    OFF(() -> 0.0),
    MOVING(() -> 0.0),
    FAST_LAUNCH(new LoggedTunableNumber("Flywheel/FastLaunch", 150)),
    SLOW_LAUNCH(new LoggedTunableNumber("Flywheel/SlowLaunch", 50)),

    // current at which the example subsystem motor moves when controlled by the operator
    MANUAL(new LoggedTunableNumber("Flywheel/OperatorCurrent", 200)),
    AUTO(() -> 0.0),

    SETPOINT(() -> 0.0),

    B4(() -> 4.0),
    B6(() -> 6.0),
    B8(() -> 8.0),
    B12(() -> 12.0);

    /** measured in rads/sec */
    private final DoubleSupplier velocityRotationsPerSec;
  }

  private final Alert flywheelMotorDisconnected;
  protected final Alert overheatAlert;

  @Getter private FlywheelState goalState = FlywheelState.OFF;
  private FlywheelState currentState = FlywheelState.OFF;
  private boolean isAtSetpoint = false;

  @Setter private double autoInput = 0.0;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    this.goalState = FlywheelState.OFF;
    this.flywheelMotorDisconnected =
        new Alert("Flywheel Motor Disconnected!", Alert.AlertType.kWarning);

    overheatAlert = new Alert("Flywheel motor overheating!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.periodicUpdates();
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel/", inputs);

    Logger.recordOutput("Flywheel/GoalState", goalState.toString());
    Logger.recordOutput("Flywheel/CurrentState", currentState.toString());
    Logger.recordOutput(
        "Flywheel/TargetVelocityRotationsPerSec", getTargetVelocityRotationsPerSec());
    Logger.recordOutput("Flywheel/CurrentVelocityRotationsPerSec", inputs.velocityRotationsPerSec);
    Logger.recordOutput("Flywheel/IsAtSetpoint", isAtSetpoint);

    flywheelMotorDisconnected.set(!inputs.motorConnected);

    handleCurrentState();
    overheatAlert.set(inputs.tempCelsius > Constants.OVERHEAT_THRESHOLD);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  /** Sets the velocity in rotations per sec */
  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void stop() {
    io.stop();
  }

  public void setGoalState(FlywheelState goalState) {
    if (this.goalState.equals(goalState)) return;

    if (goalState.equals(FlywheelState.MANUAL) && Math.abs(input) <= Constants.JOYSTICK_DEADZONE)
      return;

    this.goalState = goalState;
    switch (goalState) {
      case MANUAL -> currentState = FlywheelState.MANUAL;
      case MOVING ->
          DriverStation.reportError(
              "Flywheel: MOVING is an invalid goal state; it is a transition state!!", null);
      case OFF -> currentState = FlywheelState.OFF;
      case B4, B6, B8, B12 -> currentState = goalState;
      case AUTO -> currentState = FlywheelState.AUTO;
      default -> currentState = FlywheelState.MOVING;
    }
  }

  public boolean isAtSetpoint() {
    return EqualsUtil.epsilonEquals(
        getTargetVelocityRotationsPerSec(),
        inputs.velocityRotationsPerSec,
        Constants.Flywheel.EPSILON_VELOCITY);
  }

  private double getTargetVelocityRotationsPerSec() {
    return goalState == FlywheelState.AUTO || goalState == FlywheelState.SETPOINT
        ? autoInput
        : goalState.velocityRotationsPerSec.getAsDouble();
  }

  private void handleCurrentState() {
    isAtSetpoint = isAtSetpoint();

    switch (currentState) {
      case MOVING -> {
        setVelocity(getTargetVelocityRotationsPerSec());
        if (isAtSetpoint() && goalState != FlywheelState.AUTO) currentState = goalState;
      }
      case MANUAL -> handleManualState();
      case OFF -> stop();
      case B4, B6, B8, B12 -> runVolts(getTargetVelocityRotationsPerSec());
      case AUTO -> setVelocity(autoInput);
      default -> setVelocity(getTargetVelocityRotationsPerSec());
    }
  }

  private void handleManualState() {
    if (!goalState.equals(FlywheelState.MANUAL)) return;

    if (Math.abs(input) <= Constants.JOYSTICK_DEADZONE) {
      stop();
      return;
    }

    setVelocity(FlywheelState.MANUAL.getVelocityRotationsPerSec().getAsDouble() * input);
  }

  public void updateSetpointState(double change) {
    autoInput = inputs.setpointVelocityRotationsPerSec + change;
    goalState = FlywheelState.SETPOINT;
  }
}
