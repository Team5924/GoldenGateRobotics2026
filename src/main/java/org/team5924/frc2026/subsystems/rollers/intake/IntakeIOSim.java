package org.team5924.frc2026.subsystems.rollers.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.subsystems.rollers.generic.GenericRollerSystemIOSim;

public class IntakeIOSim extends GenericRollerSystemIOSim<IntakeIOInputs>
    implements IntakeIO {
  public IntakeIOSim() {
    super(
        DCMotor.getKrakenX60Foc(1),
        Constants.Intake.REDUCTION,
        Constants.Intake.SIM_MOI);
  }
}