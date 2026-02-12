/*
 * ObjectDetectionIO.java
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

package org.team5924.frc2026.subsystems.objectDetection;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public double testRandom = 0.0;
    public int resultsSize = 0;
    public String cameraName = "not updated yet";
    public TargetObservation latestTargetsObservation = new TargetObservation(new ArrayList<>());
    public TargetGroups latestGroupedTargets = new TargetGroups(new ArrayList<>());
    public boolean isCameraConnected = false;
    public boolean seesFuel = false;
    public int fuelCount = 0;
    public int groupCount = 0;
  }

  public static record TargetObservation(List<PhotonTrackedTarget> targets) {}

  public static record TargetGroups(List<TargetGroup> groups) {}

  public default void updateInputs(ObjectDetectionIOInputsAutoLogged inputs) {}
}
