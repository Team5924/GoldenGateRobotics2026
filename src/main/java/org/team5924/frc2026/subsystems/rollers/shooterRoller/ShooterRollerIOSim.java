package org.team5924.frc2026.subsystems.rollers.shooterRoller;

import edu.wpi.first.math.system.plant.DCMotor;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIOSim;

public class ShooterRollerIOSim extends GenericRollerSystemIOSim implements ShooterRollerIO {
  public ShooterRollerIOSim() {
    super(
        DCMotor.getKrakenX60Foc(1),
        Constants.ExampleRoller.REDUCTION,
        Constants.ExampleRoller.SIM_MOI);
  }
}
