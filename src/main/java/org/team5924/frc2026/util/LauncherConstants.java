/*
 * LauncherConstants.java
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

package org.team5924.frc2026.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.team5924.frc2026.Constants;

public class LauncherConstants {
  public static Transform3d robotToLauncherLeft =
      new Transform3d(
          Constants.TurretLeft.ROBOT_TO_TURRET, new Rotation3d(0.0, 0.0, Math.PI)); // TODO: update

  public static Transform3d robotToLauncherRight =
      new Transform3d(
          Constants.TurretRight.ROBOT_TO_TURRET, new Rotation3d(0.0, 0.0, Math.PI)); // TODO: update

  private LauncherConstants() {}
}
