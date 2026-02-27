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
    public IndexerInverseTalonFX(boolean isLeft) {
      super(
          isLeft ? Constants.IndexerLeft.CAN_ID_INVERSE : Constants.IndexerRight.CAN_ID_INVERSE,
          isLeft ? Constants.IndexerLeft.BUS : Constants.IndexerRight.BUS,
          isLeft ? Constants.IndexerLeft.CONFIG : Constants.IndexerRight.CONFIG,
          isLeft
              ? Constants.IndexerLeft.REDUCTION_INVERSE
              : Constants.IndexerRight.REDUCTION_INVERSE);
    }
  }

  // This is the other motor on indexer, the one that pushes up balls to shooter
  private final IndexerInverseTalonFX indexerInverse;

  public IndexerIOTalonFX(boolean isLeft) {
    super(
        isLeft ? Constants.IndexerLeft.CAN_ID : Constants.IndexerRight.CAN_ID,
        isLeft ? Constants.IndexerLeft.BUS : Constants.IndexerRight.BUS,
        isLeft ? Constants.IndexerLeft.CONFIG : Constants.IndexerRight.CONFIG,
        isLeft ? Constants.IndexerLeft.REDUCTION : Constants.IndexerRight.REDUCTION);
    indexerInverse = new IndexerInverseTalonFX(isLeft);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
    indexerInverse.runVolts(-volts);
  }

  @Override
  public void stop() {
    super.stop();
    indexerInverse.stop();
  }
}
