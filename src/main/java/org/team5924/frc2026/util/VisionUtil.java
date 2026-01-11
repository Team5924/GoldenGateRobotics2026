/*
 * VisionUtil.java
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

public class VisionUtil {
  /**
   * @param width width of the viewpoint in pixels
   * @param height height of the viewpoint in pixels
   * @param hFov horizontal field of view of the camera in degrees
   * @return the diagonal fov
   */
  public static double getDiagFov(double width, double height, double hFov) {
    return Math.toDegrees(
        2
            * Math.atan(
                Math.tan(Math.toRadians(hFov) / 2)
                    * Math.sqrt(width * width + height * height)
                    / width));
  }
}
