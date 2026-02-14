/*
 * AutoBuilder.java
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

package org.team5924.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.RequiredArgsConstructor;
import org.team5924.frc2026.Robot;
import org.team5924.frc2026.subsystems.drive.Drive;
import org.team5924.frc2026.subsystems.rollers.intake.Intake;
import org.team5924.frc2026.subsystems.superShooter.SuperShooter;

@RequiredArgsConstructor
public class AutoBuilder {

  private final Drive drive;
  private final SuperShooter shooter;
  // private final Climb climb;
  private final Intake intake;

  //   public Command basicDriveAuto() {
  //     return Commands.runOnce(
  //             () ->
  //                 RobotState.getInstance()
  //                     .resetPose(
  //                         AllianceFlipUtil.apply(
  //                             new Pose2d(
  //                                 RobotState.getInstance().getEstimatedPose().getTranslation(),
  //                                 Rotation2d.kPi))))
  //         .andThen(
  //             new DriveToPose(
  //                     drive,
  //                     () -> RobotState.getInstance().getEstimatedPose(),
  //                     () -> RobotState.getInstance().getEstimatedPose(),
  //                     () ->
  //                         new Translation2d((AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0) * -1.0,
  // 0.0))
  //                 .withTimeout(0.6));
  //   }

  public Command scoreAndClimbAuto() {
    return Commands.sequence(
        Robot.mAutoFactory.resetOdometry("startingPositionToHub"),
        Commands.parallel(
            Robot.mAutoFactory.trajectoryCmd("startingPositionToHub") // ,
            // AutoCommands.getShooterReady(shooter)
            ),
        AutoCommands.score(shooter).withTimeout(1.0),
        Commands.parallel(
            Robot.mAutoFactory.trajectoryCmd("HubToClimb") // ,
            // AutoCommands.getClimbReady(climb)
            ) // ,
        // AutoCommands.climb(climb)
        );
  }

  public Command scorePickupAndClimbAuto() {
    return Commands.sequence(
        Robot.mAutoFactory.resetOdometry("startingPositionToHub"),
        Commands.parallel(
            Robot.mAutoFactory.trajectoryCmd("startingPositionToHub") // ,
            // AutoCommands.getShooterReady(shooter)
            ),
        AutoCommands.score(shooter).withTimeout(1.0),
        Commands.deadline(
            Robot.mAutoFactory.trajectoryCmd("HubToDepot"), AutoCommands.intake(intake)),
        Commands.parallel(
            Robot.mAutoFactory.trajectoryCmd("DepotToHub") // ,
            // AutoCommands.getShooterReady(shooter)
            ),
        AutoCommands.score(shooter).withTimeout(1.0),
        Commands.parallel(
            Robot.mAutoFactory.trajectoryCmd("HubToClimb") // ,
            // AutoCommands.getClimbReady(climb)
            ) // ,
        // AutoCommands.climb(climb)
        );
  }
}
