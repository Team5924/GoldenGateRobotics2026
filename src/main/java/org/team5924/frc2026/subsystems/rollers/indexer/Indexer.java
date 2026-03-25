/*
 * Indexer.java
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

package org.team5924.frc2026.subsystems.rollers.indexer;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.RobotState;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRoller;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRoller.VoltageState;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class Indexer extends GenericRoller<Indexer.IndexerState> {
  @RequiredArgsConstructor
  @Getter
  public enum IndexerState implements VoltageState {
    OFF(() -> 0.0),
    INDEXING(new LoggedTunableNumber("Indexer/IndexingVoltage", 8.0));

    private final DoubleSupplier voltageSupplier;
  }

  private IndexerState goalState = IndexerState.OFF;

  public Indexer(IndexerIO indexerIO) {
    super("Indexer", indexerIO);
  }

  public void setGoalState(IndexerState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setIndexerState(goalState);
  }

  @Override
  protected void handleCurrentState() {
    if (RobotState.getInstance().isLeftFlywheelAtSetpoint())
      io.runVolts(getGoalState().getVoltageSupplier().getAsDouble());
    else io.stop();
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
