package org.team5924.frc2026.subsystems.rollers.hopperAgitator;

import org.team5924.frc2026.Constants;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIOKrakenFOC;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class HopperKrakenFOC extends GenericRollerSystemIOKrakenFOC implements HopperIO {
  /* Still undecided whether add a beambreak for if hopper is full*/

  // private static final DigitalInput fullBeamBreakSensor;

  private static final int hopperAgitatorId = Constants.HOPPER_CAN_ID;
  private static final String bus = Constants.HOPPER_BUS;
  private static final TalonFXConfiguration hopperKrakenConfig  = Constants.HOPPER_CONFIG;
  private static final double reduction = Constants.HOPPER_REDUCTION;


  public HopperKrakenFOC() {
    super(hopperAgitatorId, bus, hopperKrakenConfig, reduction);

    //fullBeamBreakSensor = Constants.HOPPER_BEAM_BREAK_ID
  }
  @Override 
  public void updateInputs(HopperIOInputs inputs) {
    super.updateInputs(inputs);    
  }
  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
  }
}
