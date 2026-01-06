/*
 * VisionIO.java
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

package org.team5924.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOCameraInputs {
    public boolean isConnected;
    public Transform3d robotToCamera;
    public int targetCount;
    public double bestTargetPoseAmbiguity;
    public double bestTargetArea;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOCameraInputs[] inputs) {}
}
