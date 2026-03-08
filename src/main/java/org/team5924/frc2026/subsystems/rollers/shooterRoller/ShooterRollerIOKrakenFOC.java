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
    public ShooterRollerFollower(boolean isLeft) {
      super(
          isLeft
              ? Constants.FlywheelFollowerLeft.CAN_ID
              : Constants.FlywheelFollowerRight.CAN_ID,
          Constants.GeneralFlywheel.BUS,
          isLeft
              ? Constants.FlywheelFollowerLeft.CONFIG
              : Constants.FlywheelFollowerRight.CONFIG,
              Constants.GeneralFlywheel.MOTOR_TO_MECHANISM);
    }
  }

  private final ShooterRollerFollower shooterFollower;

  public ShooterRollerIOKrakenFOC(boolean isLeft) {
    super(
        isLeft
            ? Constants.FlywheelLeaderLeft.CAN_ID
            : Constants.FlywheelLeaderRight.CAN_ID,
        Constants.GeneralFlywheel.BUS,
        isLeft
            ? Constants.FlywheelLeaderLeft.CONFIG
            : Constants.FlywheelLeaderRight.CONFIG,
        Constants.GeneralFlywheel.MOTOR_TO_MECHANISM);

    shooterFollower = new ShooterRollerFollower(isLeft);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
    shooterFollower.runVolts(volts);
  }

  @Override
  public void stop() {
    super.stop();
    shooterFollower.stop();
  }
}
