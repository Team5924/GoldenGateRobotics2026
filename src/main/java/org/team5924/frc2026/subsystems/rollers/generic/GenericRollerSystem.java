/*
 * GenericRollerSystem.java
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
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIO.GenericRollerSystemIOInputs;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;

@RequiredArgsConstructor
public abstract class GenericRollerSystem<
        State extends GenericRollerSystem.VoltageState,
        Inputs extends GenericRollerSystemIOInputs,
        IO extends GenericRollerSystemIO<Inputs>,
        InputsAutoLogged extends Inputs>
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

  protected final InputsAutoLogged inputs;
  protected final IO io;

  protected final Alert disconnected;
  protected final Notification disconnectedNotification;
  protected boolean wasMotorConnected = true;

  protected final Timer stateTimer = new Timer();

  public GenericRollerSystem(String name, IO io, InputsAutoLogged inputs) {
    this.name = name;
    this.io = io;
    this.inputs = inputs;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);

    disconnectedNotification =
        new Notification(
            NotificationLevel.WARNING, name + " Warning", name + " motor disconnected!");

    stateTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, (LoggableInputs) inputs);
    disconnected.set(!inputs.motorConnected);

    if (getGoalState() != lastState) {
      stateTimer.reset();
      lastState = getGoalState();
    }

    io.runVolts(getGoalState().getVoltageSupplier().getAsDouble());
    Logger.recordOutput("Rollers/" + name + "Goal", getGoalState().toString());

    if (!inputs.motorConnected && wasMotorConnected) {
      Elastic.sendNotification(disconnectedNotification);
    }
    wasMotorConnected = inputs.motorConnected;
  }
}
