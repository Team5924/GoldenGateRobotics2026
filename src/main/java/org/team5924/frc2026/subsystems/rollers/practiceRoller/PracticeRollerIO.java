package org.team5924.frc2026.subsystems.rollers.practiceRoller;

import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIO;

public interface PracticeRollerIO extends GenericRollerSystemIO<PracticeRollerIOInputs> {
  /* Run roller at volts */
  default void runVolts(double volts) {}
} 
