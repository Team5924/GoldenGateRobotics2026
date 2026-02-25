/*
 * ShooterRollerIOKrakenFOC.java
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

package org.team5924.frc2026.subsystems.rollers.shooterRoller;

import org.team5924.frc2026.Constants;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIOKrakenFOC;

public class ShooterRollerIOKrakenFOC extends GenericRollerSystemIOKrakenFOC
    implements ShooterRollerIO {

  private class ShooterRollerFollower extends GenericRollerSystemIOKrakenFOC {
    public ShooterRollerFollower(boolean left) {
      super(
          left ? Constants.ShooterRollerFollowerLeft.CAN_ID : Constants.ShooterRollerFollowerRight.CAN_ID,
          left ? Constants.ShooterRollerFollowerLeft.BUS : Constants.ShooterRollerFollowerRight.BUS,
          left ? Constants.ShooterRollerFollowerLeft.CONFIG : Constants.ShooterRollerFollowerRight.CONFIG,
          left ? Constants.ShooterRollerFollowerLeft.REDUCTION : Constants.ShooterRollerFollowerRight.REDUCTION);
    }
  }

  private final ShooterRollerFollower shooterFollower;

  public ShooterRollerIOKrakenFOC(boolean left) {
      super(
          left ? Constants.ShooterRollerLeaderLeft.CAN_ID : Constants.ShooterRollerLeaderRight.CAN_ID,
          left ? Constants.ShooterRollerLeaderLeft.BUS : Constants.ShooterRollerLeaderRight.BUS,
          left ? Constants.ShooterRollerLeaderLeft.CONFIG : Constants.ShooterRollerLeaderRight.CONFIG,
          left ? Constants.ShooterRollerLeaderLeft.REDUCTION : Constants.ShooterRollerLeaderRight.REDUCTION);

    shooterFollower = new ShooterRollerFollower(left);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
    shooterFollower.runVolts(volts);
  }
}
