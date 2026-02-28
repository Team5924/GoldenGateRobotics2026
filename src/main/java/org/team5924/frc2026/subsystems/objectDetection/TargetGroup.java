/*
 * TargetGroup.java
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class TargetGroup implements Serializable {
  public int fuelAmount;
  public List<Target> targets;
  public Target firstFiducialTarget;
  public int groupID;

  public TargetGroup(int groupID) {
    this.fuelAmount = 0;
    this.targets = new ArrayList<>();
    this.firstFiducialTarget = new Target();
    this.groupID = groupID;
  }

  public void addTarget(Target target) {
    if (targets.isEmpty()) {
      firstFiducialTarget = target;
    }
    targets.add(target);
    fuelAmount++;
  }

  /* Gets Poses of Fuel Within Group */
  public Pose2d[] getFuelPoses() {
    Pose2d[] targetPoses = new Pose2d[fuelAmount];
    for (int i = 0; i < fuelAmount; i++) {
      var target = targets.get(i).fuel;
      Translation2d targetTranslation2d =
          ObjectDetectionUtils.getRobotToTargetTranslation2d(target);
      targetPoses[i] = new Pose2d(targetTranslation2d, new Rotation2d());
    }
    return targetPoses;
  }

  public List<Translation2d> getFuelTranslations() {
    List<Translation2d> fuelPositions = new ArrayList<>();
    for (Target fuel : targets) {
      fuelPositions.add(fuel.getRobotOffset());
    }
    return fuelPositions;
  }

  public void logGroup(int id) {
    for (Target target : targets) {
      String logPath =
          "Group "
              + groupID
              + "/Target Group First Fiducial "
              + firstFiducialTarget.fuelID
              + "/Target "
              + target.fuelID
              + "/";
      target.logTarget(logPath);
    }
  }

  @Override
  public String toString() {
    StringBuilder print =
        new StringBuilder(
            "-----TARGET GROUP FIRST FIDUCIAL "
                + firstFiducialTarget.fuelID
                + "-----\nGroup Size: "
                + targets.size()
                + "\n--------------------\n");
    for (Target target : targets) {
      print.append(target.toString() + "\n");
    }
    return print.toString();
  }
}
