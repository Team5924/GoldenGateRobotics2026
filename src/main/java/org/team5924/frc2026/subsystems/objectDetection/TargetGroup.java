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
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2026.Constants;

public class TargetGroup {
  public int fuelAmount;
  public List<PhotonTrackedTarget> targets;
  public PhotonTrackedTarget firstFiducialTarget;
  public double distanceFromRobotFeet;

  public TargetGroup() {
    this.fuelAmount = 0;
    this.targets = new ArrayList<>();
    this.firstFiducialTarget = new PhotonTrackedTarget();
    this.distanceFromRobotFeet = 0.0;
  }

  public void addTarget(PhotonTrackedTarget target) {
    if (targets.isEmpty()) {
      firstFiducialTarget = target;
      distanceFromRobotFeet =
          Units.metersToFeet(
              PhotonUtils.calculateDistanceToTargetMeters(
                  Constants.ObjectDetection.CAMERA_TO_FLOOR_HEIGHT_METERS,
                  0,
                  target.getPitch(),
                  Constants.ObjectDetection.CAMERA_PITCH_RADS));
    }
    targets.add(target);
    fuelAmount++;
  }

  /* Gets Poses of Fuel Within Group */
  public Pose2d[] getFuelPoses() {
    Pose2d[] targetPoses = new Pose2d[fuelAmount];
    for (int i = 0; i < fuelAmount; i++) {
      var target = targets.get(i);
      Translation2d targetTranslation2d =
          PhotonUtils.estimateCameraToTargetTranslation(
              PhotonUtils.calculateDistanceToTargetMeters(
                  Constants.ObjectDetection.CAMERA_TO_FLOOR_HEIGHT_METERS,
                  Constants.ObjectDetection.FUEL_TOP_TO_FLOOR_METERS,
                  Constants.ObjectDetection.CAMERA_PITCH_RADS,
                  target.getPitch()),
              new Rotation2d(target.getYaw()));
      targetPoses[i] = new Pose2d(targetTranslation2d, new Rotation2d());
    }
    return targetPoses;
  }
}
