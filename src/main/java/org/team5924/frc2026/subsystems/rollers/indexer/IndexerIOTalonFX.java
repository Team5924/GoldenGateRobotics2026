/*
 * IndexerIOTalonFX.java
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

import org.team5924.frc2026.Constants;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIOKrakenFOC;

public class IndexerIOTalonFX extends GenericRollerSystemIOKrakenFOC implements IndexerIO {

  private class IndexerInverseTalonFX extends GenericRollerSystemIOKrakenFOC {
    public IndexerInverseTalonFX() {
      super(
          Constants.Indexer.CAN_ID_INVERSE,
          Constants.Indexer.BUS,
          Constants.Indexer.CONFIG,
          Constants.Indexer.REDUCTION_INVERSE);
    }
  }

  // This is the other motor on indexer, the one that pushes up balls to shooter
  private final IndexerInverseTalonFX indexerInverse;

  public IndexerIOTalonFX() {
    super(
        Constants.Indexer.CAN_ID,
        Constants.Indexer.BUS,
        Constants.Indexer.CONFIG,
        Constants.Indexer.REDUCTION);
    indexerInverse = new IndexerInverseTalonFX();
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
    indexerInverse.runVolts(-1 * volts);
  }
}
