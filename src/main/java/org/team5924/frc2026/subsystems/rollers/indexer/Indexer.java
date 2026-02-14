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
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystem.VoltageState;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIO;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIOInputsAutoLogged;
import org.team5924.frc2026.util.LoggedTunableNumber;

@Getter
public class Indexer extends GenericRollerSystem<Indexer.IndexerState> {
  @RequiredArgsConstructor
  @Getter
  public enum IndexerState implements VoltageState {
    OFF(() -> 0.0),
    INDEXING(new LoggedTunableNumber("Indexer/IndexingVoltage", 3.0));

    private final DoubleSupplier voltageSupplier;
  }

  private IndexerState goalState = IndexerState.OFF;

  // Indexer Beam Break
  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  public Indexer(IndexerIO indexerIO, BeamBreakIO beamBreakIO) {
    super("Indexer", indexerIO);
    this.beamBreakIO = beamBreakIO;
  }

  public void setGoalState(IndexerState goalState) {
    this.goalState = goalState;
    RobotState.getInstance().setIndexerState(goalState);
  }

  @Override
  public void periodic() {
    beamBreakIO.updateInputs(beamBreakInputs);
    // Note: Beam break status is logged separately via beamBreakInputs
    super.periodic();
  }
}
