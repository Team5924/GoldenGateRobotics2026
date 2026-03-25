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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class LauncherConstants {
  // TODO: update to be more accurate
  public static Transform3d robotToLauncher =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(5.764), Units.inchesToMeters(0.0), Units.inchesToMeters(21.203)),
          new Rotation3d(0.0, 0.0, Math.PI));

  private LauncherConstants() {}
}
