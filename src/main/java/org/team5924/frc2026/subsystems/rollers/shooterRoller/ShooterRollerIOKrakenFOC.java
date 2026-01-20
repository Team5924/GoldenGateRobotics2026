package org.team5924.frc2026.subsystems.rollers.shooterRoller;

import org.team5924.frc2026.Constants;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIOKrakenFOC;

public class ShooterRollerIOKrakenFOC extends GenericRollerSystemIOKrakenFOC
    implements ShooterRollerIO {

  public ShooterRollerIOKrakenFOC() {
    super(
        Constants.ExampleRoller.CAN_ID,
        Constants.ExampleRoller.BUS,
        Constants.ExampleRoller.CONFIG,
        Constants.ExampleRoller.REDUCTION);
  }

  public void updateInputs(ShooterRollerIOInputs inputs) {
    super.updateInputs(inputs);
  }

  @Override
  public void runVolts(double volts) {
    super.runVolts(volts);
  }
}
