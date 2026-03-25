/*
 * IntakeIOTalonFX.java
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

package org.team5924.frc2026.subsystems.rollers.intake;

import org.team5924.frc2026.Constants.Intake;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerIOTalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeIOTalonFX extends GenericRollerIOTalonFX implements IntakeIO {
  private class IntakeFollowerIOTalonFX extends GenericRollerIOTalonFX {
    public IntakeFollowerIOTalonFX(int id, String bus, TalonFXConfiguration config, double reduction) {
      super(id, bus, config, reduction);
    }
  }

  private final IntakeFollowerIOTalonFX follower;

  public IntakeIOTalonFX() {
    super(Intake.CAN_ID, Intake.BUS, Intake.CONFIG, Intake.MOTOR_TO_MECHANISM);
    follower = new IntakeFollowerIOTalonFX(Intake.FOLLOWER_CAN_ID, Intake.BUS, Intake.CONFIG, Intake.MOTOR_TO_MECHANISM);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
    follower.runVolts(-volts);
  }

  @Override
  public void stop() {
    super.stop();
    follower.stop();
  }
}
