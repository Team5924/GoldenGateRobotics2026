/*
 * ShooterRollerIOTalonFX.java
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

import org.team5924.frc2026.Constants.FlywheelFollowerLeft;
import org.team5924.frc2026.Constants.FlywheelFollowerRight;
import org.team5924.frc2026.Constants.FlywheelLeaderLeft;
import org.team5924.frc2026.Constants.FlywheelLeaderRight;
import org.team5924.frc2026.Constants.GeneralFlywheel;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerIOTalonFX;

public class ShooterRollerIOTalonFX extends GenericRollerIOTalonFX implements ShooterRollerIO {

  private class ShooterRollerFollower extends GenericRollerIOTalonFX {
    public ShooterRollerFollower(boolean isLeft) {
      super(
          isLeft ? FlywheelFollowerLeft.CAN_ID : FlywheelFollowerRight.CAN_ID,
          GeneralFlywheel.BUS,
          isLeft ? FlywheelFollowerLeft.CONFIG : FlywheelFollowerRight.CONFIG,
          GeneralFlywheel.MOTOR_TO_MECHANISM);
    }
  }

  private final ShooterRollerFollower shooterFollower;

  public ShooterRollerIOTalonFX(boolean isLeft) {
    super(
        isLeft ? FlywheelLeaderLeft.CAN_ID : FlywheelLeaderRight.CAN_ID,
        GeneralFlywheel.BUS,
        isLeft ? FlywheelLeaderLeft.CONFIG : FlywheelLeaderRight.CONFIG,
        GeneralFlywheel.MOTOR_TO_MECHANISM);

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
