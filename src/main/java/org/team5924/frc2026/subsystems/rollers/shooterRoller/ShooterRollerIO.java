package org.team5924.frc2026.subsystems.rollers.shooterRoller;

import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIO;

public interface ShooterRollerIO extends GenericRollerSystemIO {
  @AutoLog
  public class ShooterRollerIOInputs extends GenericRollerSystemIOInputs {}

  /* Run roller at volts */
  default void runVolts(double volts) {}
}
