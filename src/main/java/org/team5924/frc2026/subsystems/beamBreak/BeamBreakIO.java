package org.team5924.frc2026.subsystems.beamBreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  @AutoLog
  class BeamBreakIOInputs {
    public BeamBreakIOData data = new BeamBreakIOData(false);
  }

  record BeamBreakIOData(boolean broken) {}

  default void updateInputs(BeamBreakIOInputs inputs) {}    
}
