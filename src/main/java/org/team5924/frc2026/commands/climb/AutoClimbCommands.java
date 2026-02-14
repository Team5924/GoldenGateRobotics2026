/*
 * AutoClimbCommands.java
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

package org.team5924.frc2026.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team5924.frc2026.subsystems.drive.Drive;

public class AutoClimbCommands {
  // TODO Make and auto climb program
  // public static Command autoClimb(
  //   Drive drive,
  //   SuperShooter shooter,
  //   DoubleSupplier driverX,
  //   DoubleSupplier driverY,
  //   DoubleSupplier driverOmega,
  //   Command joystickDrive,
  //   Supplier<Command> controllerRumble,
  //   BooleanSupplier trigger,
  //   BooleanSupplier disableReefAutoAlign,
  //   BooleanSupplier manualEject) {
  //   }
  private AutoClimbCommands() {}

  public static Command autoClimb(Drive drive, /*Climb climb, */ BooleanSupplier s) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'autoClimb'");
  }
}
