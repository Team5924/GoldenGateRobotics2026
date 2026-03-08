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
import org.team5924.frc2026.commands.shooter.AutoScoreCommands;
import org.team5924.frc2026.generated.TunerConstants;
import org.team5924.frc2026.subsystems.drive.Drive;
import org.team5924.frc2026.subsystems.drive.GyroIO;
import org.team5924.frc2026.subsystems.drive.GyroIOPigeon2;
import org.team5924.frc2026.subsystems.drive.GyroIOSim;
import org.team5924.frc2026.subsystems.drive.ModuleIO;
import org.team5924.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.team5924.frc2026.subsystems.drive.ModuleIOTalonFXSim;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot.IntakePivotState;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIO;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIOSim;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIOTalonFX;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood.ShooterHoodState;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIO;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIOSim;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIOTalonFX;
import org.team5924.frc2026.subsystems.rollers.hopper.Hopper;
import org.team5924.frc2026.subsystems.rollers.hopper.HopperIO;
import org.team5924.frc2026.subsystems.rollers.hopper.HopperKrakenFOC;
import org.team5924.frc2026.subsystems.rollers.indexer.Indexer;
import org.team5924.frc2026.subsystems.rollers.indexer.IndexerIO;
import org.team5924.frc2026.subsystems.rollers.indexer.IndexerIOTalonFX;
import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.rollers.intake.Intake.IntakeState;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIO;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIOKrakenFOC;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIOSim;
import org.team5924.frc2026.subsystems.rollers.shooterFlywheel.ShooterFlywheel;
import org.team5924.frc2026.subsystems.rollers.shooterFlywheel.ShooterFlywheel.ShooterFlywheelState;
import org.team5924.frc2026.subsystems.rollers.shooterFlywheel.ShooterFlywheelIO;
import org.team5924.frc2026.subsystems.rollers.shooterFlywheel.ShooterFlywheelKrakenFOC;
import org.team5924.frc2026.subsystems.turret.Turret;
import org.team5924.frc2026.subsystems.turret.Turret.TurretState;
import org.team5924.frc2026.subsystems.turret.TurretIO;
import org.team5924.frc2026.subsystems.turret.TurretIOSim;
import org.team5924.frc2026.subsystems.turret.TurretIOTalonFX;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;
  private final Intake intake;
  private final IntakePivot intakePivot;
  private final Hopper hopper;
  private final Indexer indexer;

  private final ShooterHood shooterHoodRight;
  private final ShooterFlywheel shooterFlywheelRight;
  private final Turret turretRight;

  private final ShooterHood shooterHoodLeft;
  private final ShooterFlywheel shooterFlywheelLeft;
  private final Turret turretLeft;
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

                
        intake = new Intake(new IntakeIOKrakenFOC());
        intakePivot = new IntakePivot(new IntakePivotIOTalonFX());
        hopper = new Hopper(new HopperKrakenFOC());
        indexer = new Indexer(new IndexerIOTalonFX());

        shooterHoodLeft = new ShooterHood(new ShooterHoodIOTalonFX(true), true);
        shooterFlywheelLeft = new ShooterFlywheel(new ShooterFlywheelKrakenFOC(true), true);
        turretLeft = new Turret(new TurretIOTalonFX(true), true);

        shooterHoodRight = new ShooterHood(new ShooterHoodIOTalonFX(false), false);
        shooterFlywheelRight = new ShooterFlywheel(new ShooterFlywheelKrakenFOC(false), false);
        turretRight = new Turret(new TurretIOTalonFX(false), false);

        
        // intake = new Intake(new IntakeIO() {});
        // intakePivot = new IntakePivot(new IntakePivotIO() {});
        // hopper = new Hopper(new HopperIO() {});
        // indexer = new Indexer(new IndexerIO() {});

        // shooterHoodLeft = new ShooterHood(new ShooterHoodIO() {}, true);
        // shooterFlywheelLeft = new ShooterFlywheel(new ShooterFlywheelIO() {}, true);
        // turretLeft = new Turret(new TurretIO() {}, true);

        // shooterHoodRight = new ShooterHood(new ShooterHoodIO() {}, false);
        // shooterFlywheelRight = new ShooterFlywheel(new ShooterFlywheelIO() {}, false);
        // turretRight = new Turret(new TurretIO() {}, false);
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

        intake = new Intake(new IntakeIOSim());
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        hopper = new Hopper(new HopperIO() {}); // TODO: Hopper sim implementation
        indexer = new Indexer(new IndexerIO() {});

        shooterHoodLeft = new ShooterHood(new ShooterHoodIOSim(true), true);
        shooterFlywheelLeft = null;
        // new ShooterFlywheel(new ShooterFlywheelIOSim(true), true);
        turretLeft = new Turret(new TurretIOSim(true), true);

        shooterHoodRight = new ShooterHood(new ShooterHoodIOSim(false), false);
        shooterFlywheelRight = null;
        // new ShooterFlywheel(new ShooterFlywheelIOSim(false), false);
        turretRight = new Turret(new TurretIOSim(false), false);

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

        intake = new Intake(new IntakeIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        hopper = new Hopper(new HopperIO() {}); // TODO: Add replay IO implementation
        indexer = new Indexer(new IndexerIO() {});

        shooterHoodLeft = new ShooterHood(new ShooterHoodIO() {}, true);
        shooterFlywheelLeft = new ShooterFlywheel(new ShooterFlywheelIO() {}, true);
        turretLeft = new Turret(new TurretIO() {}, true);

        shooterHoodRight = new ShooterHood(new ShooterHoodIO() {}, false);
        shooterFlywheelRight = new ShooterFlywheel(new ShooterFlywheelIO() {}, false);
        turretRight = new Turret(new TurretIO() {}, false);
        break;
    }

    // Auto commands
    NamedCommands.registerCommand(
        "Run Shooter",
        Commands.runOnce(
            () -> {
              // AutoScoreCommands.autoScore(drive, shooter);
            }));

    NamedCommands.registerCommand(
        "Run L1 Climb",
        Commands.runOnce(
            () -> {
              // add once climb is figured out
            }));

    // TODO: Uncomment when intake subsystem is enabled
    NamedCommands.registerCommand(
        "Run Intake",
        Commands.runOnce(
            () -> {
              // intake.setGoalState(IntakeState.INTAKE);
            }));

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
    // // Default command, normal field-relative drive
    if (Constants.currentMode == Constants.Mode.SIM) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -driveController.getRawAxis(0),
              () -> -driveController.getLeftY(),
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
    // driveController
    //     .leftTrigger()
    //     .whileTrue(
    //         Commands.runOnce(
    //             () -> intake.setGoalState(IntakeState.INTAKE)
    //     ));

    // driveController
    //     .leftTrigger()
    //     .whileFalse(
    //         Commands.runOnce(
    //             () -> intake.setGoalState(IntakeState.OFF)
    //     ));

    hopper.setDefaultCommand(
        Commands.run(() -> hopper.setGoalState(Hopper.HopperState.ON), hopper));
    shooterFlywheelLeft.setDefaultCommand(
        Commands.runOnce(
            () -> shooterFlywheelLeft.setGoalState(ShooterFlywheelState.OFF), shooterFlywheelLeft));
    shooterFlywheelRight.setDefaultCommand(
        Commands.runOnce(
            () -> shooterFlywheelRight.setGoalState(ShooterFlywheelState.OFF), shooterFlywheelRight));

    driveController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakePivot.setGoalState(IntakePivotState.DOWN);
                  intake.setGoalState(IntakeState.INTAKE);
                },
                intakePivot,
                hopper,
                intake));
    driveController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakePivot.setGoalState(IntakePivotState.STOW);
                  intake.setGoalState(IntakeState.OFF);
                },
                intakePivot,
                hopper,
                intake));

    driveController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakePivot.setGoalState(IntakePivotState.DOWN);
                  hopper.setGoalState(Hopper.HopperState.SPIT);
                  intake.setGoalState(IntakeState.SPITOUT);
                },
                intakePivot,
                hopper,
                intake));
    driveController
        .leftTrigger()
        .onFalse(
            Commands.runOnce(
                () -> {
                  intakePivot.setGoalState(IntakePivotState.STOW);
                  hopper.setGoalState(Hopper.HopperState.ON);
                  intake.setGoalState(IntakeState.OFF);
                },
                intakePivot,
                hopper,
                intake));
    driveController
        .rightTrigger()
        .whileTrue(
            Commands.runOnce(
                () -> {
                  intakePivot.setGoalState(IntakePivotState.SHOOTING);
                  indexer.setGoalState(Indexer.IndexerState.INDEXING);
                },
                intakePivot,
                shooterFlywheelLeft,
                shooterFlywheelRight,
                indexer));

    operatorController
        .a()
        .whileTrue(
            Commands.runOnce(
                () -> {
                  shooterFlywheelRight.setGoalState(ShooterFlywheelState.B4);
                  shooterFlywheelLeft.setGoalState(ShooterFlywheelState.B4);
                },
                shooterFlywheelLeft,
                shooterFlywheelRight));

    operatorController
        .b()
        .whileTrue(
            Commands.runOnce(
                () -> {
                  shooterFlywheelRight.setGoalState(ShooterFlywheelState.B6);
                  shooterFlywheelLeft.setGoalState(ShooterFlywheelState.B6);
                },
                shooterFlywheelLeft,
                shooterFlywheelRight));

    operatorController
        .x()
        .whileTrue(
            Commands.runOnce(
                () -> {
                  shooterFlywheelRight.setGoalState(ShooterFlywheelState.B8);
                  shooterFlywheelLeft.setGoalState(ShooterFlywheelState.B8);
                },
                shooterFlywheelLeft,
                shooterFlywheelRight));

    operatorController
        .y()
        .whileTrue(
            Commands.runOnce(
                () -> {
                  shooterFlywheelRight.setGoalState(ShooterFlywheelState.B12);
                  shooterFlywheelLeft.setGoalState(ShooterFlywheelState.B12);
                },
                shooterFlywheelLeft,
                shooterFlywheelRight));

    // operatorController
    //     .rightTrigger()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               shooterFlywheelRight.setGoalState(ShooterFlywheelState.OFF);
    //               shooterFlywheelLeft.setGoalState(ShooterFlywheelState.OFF);
    //             },
    //             shooterFlywheelLeft,
    //             shooterFlywheelRight));

    operatorController
        .leftTrigger()
        .whileTrue(
            AutoScoreCommands.runTrackTargetCommand(
                turretLeft, shooterHoodLeft, shooterFlywheelLeft, true));
    operatorController
        .leftTrigger()
        .whileTrue(
            AutoScoreCommands.runTrackTargetCommand(
                turretRight, shooterHoodRight, shooterFlywheelRight, false));

    // ---------------------------------------------------------

    // turretLeft.setDefaultCommand(
    //     Commands.run(
    //         () -> {
    //           turretLeft.setGoalState(TurretState.MANUAL);
    //           turretLeft.setInput(driveController.getRightX());
    //         },
    //         turretLeft));

    driveController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  turretLeft.setGoalState(TurretState.ZERO);
                },
                turretLeft));

    driveController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  turretLeft.setGoalState(TurretState.NINETY);
                },
                turretLeft));

    // ---------------------------------------------------------

    shooterHoodLeft.setDefaultCommand(
        Commands.run(
            () -> {
              shooterHoodLeft.setGoalState(ShooterHoodState.MANUAL);
              shooterHoodLeft.setInput(operatorController.getRightY());
              shooterHoodRight.setGoalState(ShooterHoodState.MANUAL);
              shooterHoodRight.setInput(operatorController.getRightY());
            },
            shooterHoodLeft,
            shooterHoodRight));
    operatorController
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooterHoodLeft.setGoalState(ShooterHoodState.BUMPER_SHOOTING);
                },
                shooterHoodLeft));

    operatorController
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooterHoodLeft.setGoalState(ShooterHoodState.ZERO);
                },
                shooterHoodLeft));

    // driveController
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.runOnce(
    //             () -> {
    //               // shooterHoodLeft.setGoalState(ShooterHoodState.OFF);
    //               // shooterHoodRight.setGoalState(ShooterHoodState.OFF);
    //               // turretLeft.setGoalState(Turret.TurretState.OFF);
    //               // turretRight.setGoalState(Turret.TurretState.OFF);
    //               shooterFlywheelLeft.setGoalState(ShooterFlywheelState.BUMPER_SHOOTING);
    //               shooterFlywheelRight.setGoalState(ShooterFlywheelState.BUMPER_SHOOTING);
    //             },
    //             shooterHoodLeft,
    //             shooterHoodRight,
    //             turretLeft,
    //             turretRight,
    //             shooterFlywheelLeft,
    //             shooterFlywheelRight));

    // operatorController
    //     .leftBumper()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               hopper.setGoalState(HopperState.ON);
    //               indexer.setGoalState(IndexerState.INDEXING);
    //               shooterFlywheelLeft.setGoalState(ShooterFlywheelState.BUMPER_SHOOTING);
    //               shooterFlywheelRight.setGoalState(ShooterFlywheelState.BUMPER_SHOOTING);
    //             }));

    // operatorController
    //     .leftBumper()
    //     .onFalse(
    //         Commands.runOnce(
    //             () -> {
    //               hopper.setGoalState(HopperState.OFF);
    //               indexer.setGoalState(IndexerState.OFF);
    //               shooterFlywheelLeft.setGoalState(ShooterFlywheelState.OFF);
    //               shooterFlywheelRight.setGoalState(ShooterFlywheelState.OFF);
    //             }));

    // operatorController
    //     .leftTrigger()
    //     .onTrue(Commands.runOnce(() -> intake.setGoalState(IntakeState.INTAKE)));
    // operatorController
    //     .leftTrigger()
    //     .onFalse(Commands.runOnce(() -> intake.setGoalState(IntakeState.OFF)));

    operatorController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooterFlywheelRight.setGoalState(ShooterFlywheelState.MANUAL);
                }));

    operatorController
        .rightBumper()
        .onFalse(
            Commands.runOnce(
                () -> {
                  shooterFlywheelRight.setGoalState(ShooterFlywheelState.OFF);
                }));

    // shooterHoodRight.setDefaultCommand(
    //     ShooterCommands.manualShooterHood(shooterHoodRight, () ->
    // operatorController.getRightY()));
    // shooterHoodLeft.setDefaultCommand(
    //     ShooterCommands.manualShooterHood(shooterHoodLeft, () ->
    // operatorController.getRightY()));
    // turretRight.setDefaultCommand(
    //     ShooterCommands.manualTurret(turretRight, () -> operatorController.getLeftX()));
    // turretLeft.setDefaultCommand(
    //     ShooterCommands.manualTurret(turretLeft, () -> operatorController.getRightX()));

    // shooterHoodLeft.setDefaultCommand(getAutonomousCommand()); // todo
    // shooterHoodRight.setDefaultCommand(getAutonomousCommand()); // todo
    // turretLeft.setDefaultCommand(getAutonomousCommand()); // todo
    // turretRight.setDefaultCommand(getAutonomousCommand()); // todo
    // shooterFlywheelLeft.setDefaultCommand(getAutonomousCommand()); // todo
    // shooterFlywheelRight.setDefaultCommand(getAutonomousCommand()); // todo

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
