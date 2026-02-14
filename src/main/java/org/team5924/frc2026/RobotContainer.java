/*
 * RobotContainer.java
 */

/* 
 * Copyright (C) 2025-2026 Team 5924 - Golden Gate Robotics and/or its affiliates.
 *
 * This file, and the associated project, are offered under the GNU General
 * Public License v3.0. A copy of this license can be found in LICENSE.md
 * at the root of this project.
 *
 * If this file has been separated from the original project, you should have
 * received a copy of the GNU General Public License along with it.
 * If you did not, see <https://www.gnu.org/licenses>.
 */

package org.team5924.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team5924.frc2026.commands.drive.DriveCommands;
import org.team5924.frc2026.generated.TunerConstants;
import org.team5924.frc2026.subsystems.drive.Drive;
import org.team5924.frc2026.subsystems.drive.GyroIO;
import org.team5924.frc2026.subsystems.drive.GyroIOPigeon2;
import org.team5924.frc2026.subsystems.drive.GyroIOSim;
import org.team5924.frc2026.subsystems.drive.ModuleIO;
import org.team5924.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.team5924.frc2026.subsystems.drive.ModuleIOTalonFXSim;
import org.team5924.frc2026.subsystems.rollers.hopperAgitator.Hopper;
import org.team5924.frc2026.subsystems.rollers.hopperAgitator.HopperIO;
import org.team5924.frc2026.subsystems.rollers.hopperAgitator.HopperKrakenFOC;
import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIO;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIOKrakenFOC;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIOSim;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRoller;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRollerIO;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRollerIOKrakenFOC;
import org.team5924.frc2026.subsystems.rollers.shooterRoller.ShooterRollerIOSim;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIO;
import org.team5924.frc2026.subsystems.sensors.BeamBreakIOHardware;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHoodIO;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHoodIOSim;
import org.team5924.frc2026.subsystems.shooterHood.ShooterHoodIOTalonFX;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;
  private final SuperShooter shooter;
  private final ShooterHood shooterHood;
  private final ShooterRoller shooterRoller;
  private final Intake intake;
  private final Hopper hopper;

  // private final ExampleSystem exampleSystem;
  // private final ExampleRoller exampleRoller;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (pose) -> {});

        shooterHood = new ShooterHood(new ShooterHoodIOTalonFX());
        shooterRoller =
            new ShooterRoller(
                new ShooterRollerIOKrakenFOC(),
                new BeamBreakIOHardware(Constants.ShooterRoller.BEAM_BREAK_PORT));
        intake = new Intake(new IntakeIOKrakenFOC());
        shooter = new SuperShooter(shooterRoller, shooterHood);
        hopper = new Hopper(new HopperKrakenFOC(), new BeamBreakIOHardware(0));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        // vision = new Vision(drive,
        // new VisionIOPhotonVisionSim(
        // camera0Name, robotToCamera0,
        // driveSimulation::getSimulatedDriveTrainPose),
        // new VisionIOPhotonVisionSim(
        // camera0Name, robotToCamera0,
        // driveSimulation::getSimulatedDriveTrainPose);

        shooterHood = new ShooterHood(new ShooterHoodIOSim());
        shooterRoller = new ShooterRoller(new ShooterRollerIOSim(), new BeamBreakIO() {});
        intake = new Intake(new IntakeIOSim());
        shooter = new SuperShooter(shooterRoller, shooterHood);
        hopper = new Hopper(null, new BeamBreakIO() {}); // TODO: Hopper sim implementation
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});

        shooterHood = new ShooterHood(new ShooterHoodIO() {});
        shooterRoller = new ShooterRoller(new ShooterRollerIO() {}, new BeamBreakIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new SuperShooter(shooterRoller, shooterHood);
        hopper =
            new Hopper(
                new HopperIO() {}, new BeamBreakIO() {}); // TODO: Add replay IO implementation
        // vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Auto commands
    // NamedCommands.registerCommand(
    //     "Run Shooter",
    //     Commands.runOnce(
    //         () -> {
    //           shooter.setGoalState(ShooterState.AUTO_SHOOTING);
    //           // AutoScoreCommands.autoScore(drive, shooter);
    //         }));

    NamedCommands.registerCommand(
        "Run L1 Climb",
        Commands.runOnce(
            () -> {
              // add once climb is figured out
            }));

    // TODO: Uncomment when intake subsystem is enabled
    // NamedCommands.registerCommand(
    //     "Run Intake",
    //     Commands.runOnce(
    //         () -> {
    //           intake.setGoalState(IntakeState.INTAKE);
    //         }));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    if (Constants.currentMode == Constants.Mode.SIM) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -driveController.getLeftY(),
              () -> -driveController.getRawAxis(0),
              () -> -driveController.getRawAxis(2)));
    } else {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -driveController.getLeftY(),
              () -> -driveController.getLeftX(),
              () -> -driveController.getRightX()));
    }
    // [driver] SLOW MODE YIPE
    driveController
        .y()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * Constants.SLOW_MODE_MULTI,
                () -> -driveController.getLeftX() * Constants.SLOW_MODE_MULTI,
                () -> -driveController.getRightX() * Constants.SLOW_MODE_MULTI));

    // [driver] 0-DEGREE MODE
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> Rotation2d.kZero));

    // [driver] Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // [driver] Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    driveController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // // [operator] press a -> deploy example subystem up
    // operatorController
    // .a()
    // .onTrue(Commands.runOnce(() ->
    // exampleSystem.setGoalState(ExampleSystemState.UP)));

    // // [operator] release a -> stow example subystem
    // operatorController
    // .a()
    // .onFalse(Commands.runOnce(() ->
    // exampleSystem.setGoalState(ExampleSystemState.STOW)));

    // // [operator] press b -> run example roller
    // operatorController
    // .b()
    // .onTrue(Commands.runOnce(() ->
    // exampleRoller.setGoalState(ExampleRollerState.INTAKE)));

    // // [operator] release b -> run example roller
    // operatorController
    // .b()
    // .onFalse(Commands.runOnce(() ->
    // exampleRoller.setGoalState(ExampleRollerState.IDLE)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
