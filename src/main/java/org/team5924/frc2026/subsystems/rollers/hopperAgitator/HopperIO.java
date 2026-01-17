package org.team5924.frc2026.subsystems.rollers.hopperAgitator;

import org.littletonrobotics.junction.AutoLog;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIO;

public interface HopperIO extends GenericRollerSystemIO{
  @AutoLog
  public static class HopperIOInputs extends GenericRollerSystemIOInputs{
    //public boolean isFull = false; 
  }


  // Updates motor (and beam break if we add) inputs 
  public default void updateInputs(HopperIOInputs inputs){}
  // Runs Motor at inputted volts
  default void runVolts(double volts){}
}

