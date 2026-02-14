package org.team5924.frc2026.commands;

import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.rollers.intake.Intake.IntakeState;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoCommands {
    public static Command intake(Intake intake) {
        return Commands.run(() -> intake.setGoalState(IntakeState.INTAKE), intake);
    }

    // this command will put shooter into place for shooting, so it needs turret and rotation data idk????

    // public static Command getShooterReady(SuperShooter shooter) {
    //     return Commands.run(() -> shooter.setGoalState(ShooterState.AUTO_SHOOTING), shooter);
    // }

    public static Command score(SuperShooter shooter) {
        return Commands.run(() -> shooter.setGoalState(ShooterState.AUTO_SHOOTING), shooter);
    }
}
