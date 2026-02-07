package org.team5924.frc2026.subsystems.beamBreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOBeamBreak implements BeamBreakIO{
    private final DigitalInput beamBreak;
    
    public BeamBreakIOBeamBreak(int id) {
        beamBreak = new DigitalInput(id);
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.data = new BeamBreakIOData(beamBreak.get());
  }
}

