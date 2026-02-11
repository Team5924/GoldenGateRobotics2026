package org.team5924.frc2026.subsystems.rollers.practiceRoller;

import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIOKrakenFOC;

import org.team5924.frc2026.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

public class PracticeRollerIOKrakenFOC extends GenericRollerSystemIOKrakenFOC<PracticeRollerIOInputs>
    implements PracticeRollerIO {
    
  private final DigitalInput beamBreak = new DigitalInput(Constants.PracticeRoller.BEAM_BREAK_ID);

  public PracticeRollerIOKrakenFOC() {
    super(
      Constants.PracticeRoller.CAN_ID,
      Constants.PracticeRoller.BUS,
      Constants.PracticeRoller.CONFIG,
      Constants.PracticeRoller.REDUCTION);
  }

  @Override
  public void updateInputs(PracticeRollerIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.hasFuel = beamBreak.get();
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
  }
}
