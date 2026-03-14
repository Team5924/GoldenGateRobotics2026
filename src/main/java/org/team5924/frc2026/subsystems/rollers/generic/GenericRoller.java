/*
 * GenericRoller.java
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

package org.team5924.frc2026.subsystems.rollers.generic;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;

public abstract class GenericRoller<State extends GenericRoller.VoltageState>
    extends SubsystemBase {
  public interface VoltageState {
    DoubleSupplier getVoltageSupplier();

    default DoubleSupplier getHandoffVoltage() {
      return getVoltageSupplier();
    }
  }

  public abstract State getGoalState();

  private State lastState;

  protected final String name;

  protected final GenericRollerIO io;
  protected final GenericRollerIOInputsAutoLogged inputs = new GenericRollerIOInputsAutoLogged();

  protected final Alert disconnectedAlert;
  protected final Alert overheatAlert;

  protected final Timer stateTimer = new Timer();

  public GenericRoller(String name, GenericRollerIO io) {
    this.name = name;
    this.io = io;

    disconnectedAlert = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    overheatAlert = new Alert(name + " motor overheating!", Alert.AlertType.kWarning);

    stateTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnectedAlert.set(!inputs.motorConnected);
    overheatAlert.set(inputs.tempCelsius > Constants.OVERHEAT_THRESHOLD);

    if (getGoalState() != lastState) {
      stateTimer.reset();
      lastState = getGoalState();
    }

    handleCurrentState();
    Logger.recordOutput("Rollers/" + name + "/Goal", getGoalState().toString());
  }

  protected void handleCurrentState() {
    runVolts(getGoalState().getVoltageSupplier().getAsDouble());
  }

  private void runVolts(double volts) {
    io.runVolts(volts);
  }
}
