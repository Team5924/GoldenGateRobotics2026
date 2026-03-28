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
import org.team5924.frc2026.subsystems.flywheel.Flywheel;
import org.team5924.frc2026.subsystems.flywheel.FlywheelIO;
import org.team5924.frc2026.subsystems.flywheel.FlywheelIOSim;
import org.team5924.frc2026.subsystems.flywheel.FlywheelIOTalonFX;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot.IntakePivotState;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIO;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIOSim;
import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivotIOTalonFX;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHood;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIO;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIOSim;
import org.team5924.frc2026.subsystems.pivots.shooterHood.ShooterHoodIOTalonFX;
import org.team5924.frc2026.subsystems.rollers.hopper.Hopper;
import org.team5924.frc2026.subsystems.rollers.hopper.HopperIO;
import org.team5924.frc2026.subsystems.rollers.hopper.HopperIOSim;
import org.team5924.frc2026.subsystems.rollers.hopper.HopperIOTalonFX;
import org.team5924.frc2026.subsystems.rollers.indexer.Indexer;
import org.team5924.frc2026.subsystems.rollers.indexer.IndexerIO;
import org.team5924.frc2026.subsystems.rollers.indexer.IndexerIOSim;
import org.team5924.frc2026.subsystems.rollers.indexer.IndexerIOTalonFX;
import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.rollers.intake.Intake.IntakeState;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIO;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIOSim;
import org.team5924.frc2026.subsystems.rollers.intake.IntakeIOTalonFX;
import org.team5924.frc2026.subsystems.vision.Vision;
import org.team5924.frc2026.subsystems.vision.VisionConstants;
import org.team5924.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.team5924.frc2026.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;

  private final Vision vision;

  private final Intake intake;
  private final IntakePivot intakePivot;
  private final Hopper hopper;
  private final Indexer indexer;

  private final ShooterHood shooterHood;
  private final Flywheel flywheel;

  // Real/IO implementation
  private final boolean realDrive = true;
  private final boolean realVision = true;
  private final boolean realIntake = true;
  private final boolean realIntakePivot = true;
  private final boolean realHopper = true;
  private final boolean realIndexer = true;

  private final boolean realShooterHood = true;
  private final boolean realFlywheel = true;

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
            realDrive
                ? new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight),
                    (pose) -> {})
                : new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    (pose) -> {});

        vision =
            realVision
                ? new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonVision(
                        VisionConstants.FRONT_LEFT_NAME, VisionConstants.FRONT_LEFT_TRANSFORM),
                    new VisionIOPhotonVision(
                        VisionConstants.FRONT_RIGHT_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM))
                : null;

        intake = realIntake ? new Intake(new IntakeIOTalonFX()) : new Intake(new IntakeIO() {});
        intakePivot =
            realIntakePivot
                ? new IntakePivot(new IntakePivotIOTalonFX())
                : new IntakePivot(new IntakePivotIO() {});
        hopper = realHopper ? new Hopper(new HopperIOTalonFX()) : new Hopper(new HopperIO() {});

        indexer =
            realIndexer ? new Indexer(new IndexerIOTalonFX()) : new Indexer(new IndexerIO() {});
        shooterHood =
            realShooterHood
                ? new ShooterHood(new ShooterHoodIOTalonFX())
                : new ShooterHood(new ShooterHoodIO() {});
        flywheel =
            realFlywheel
                ? new Flywheel(new FlywheelIOTalonFX())
                : new Flywheel(new FlywheelIO() {});
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.FRONT_LEFT_NAME,
                    VisionConstants.FRONT_LEFT_TRANSFORM,
                    VisionConstants.SIM_THRIFTYCAM_PROPERTIES,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.FRONT_RIGHT_NAME,
                    VisionConstants.FRONT_RIGHT_TRANSFORM,
                    VisionConstants.SIM_THRIFTYCAM_PROPERTIES,
                    driveSimulation::getSimulatedDriveTrainPose));

        intake = new Intake(new IntakeIOSim());
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        hopper = new Hopper(new HopperIOSim());

        indexer = new Indexer(new IndexerIOSim());
        shooterHood = new ShooterHood(new ShooterHoodIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());

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

        vision = null;

        intake = new Intake(new IntakeIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        hopper = new Hopper(new HopperIO() {});

        indexer = new Indexer(new IndexerIO() {});
        shooterHood = new ShooterHood(new ShooterHoodIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
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
    // Default command, normal field-relative drive
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
        .onTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * Constants.SLOW_MODE_MULTI,
                () -> -driveController.getLeftX() * Constants.SLOW_MODE_MULTI,
                () -> -driveController.getRightX() * Constants.SLOW_MODE_MULTI));

    // [driver] 0-DEGREE MODE
    driveController
        .a()
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> Rotation2d.kZero));

    // [driver] Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose
            // during simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    driveController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // [driver] Reset gyro to 0° when B button is pressed
    driveController.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // ### hopper on by default
    hopper.setDefaultCommand(
        Commands.run(() -> hopper.setGoalState(Hopper.HopperState.ON), hopper));

    /* ### intake ### */
    driveController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakePivot.setGoalState(IntakePivotState.DOWN);
                  intake.setGoalState(IntakeState.INTAKE);
                },
                intakePivot,
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
                intake));

    // manual intake
    intakePivot.setDefaultCommand(
        Commands.run(
            () -> {
              intakePivot.setGoalState(IntakePivotState.MANUAL);
              intakePivot.setInput(operatorController.getLeftX());
            },
            intakePivot));


    // shooter
    driveController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  flywheel.setGoalState(Flywheel.FlywheelState.B8);
                  indexer.setGoalState(Indexer.IndexerState.INDEXING);
                },
                flywheel,
                indexer));

    driveController
        .leftBumper()
        .onFalse(
            Commands.runOnce(
                () -> {
                  flywheel.setGoalState(Flywheel.FlywheelState.OFF);
                  indexer.setGoalState(Indexer.IndexerState.OFF);
                },
                flywheel,
                indexer));

    shooterHood.setDefaultCommand(
        Commands.runOnce(
            () -> shooterHood.setGoalState(ShooterHood.ShooterHoodState.MANUAL), shooterHood));

    driveController
        .rightStick()
        .onTrue(
            Commands.run(() -> shooterHood.setInput(() -> driveController.getRightY()), shooterHood));

    // TODO: auto shooting
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
