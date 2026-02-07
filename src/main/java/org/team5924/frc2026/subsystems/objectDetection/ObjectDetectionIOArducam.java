/*
 * ObjectDetectionIOArducam.java
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

import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2026.Constants;

public class ObjectDetectionIOArducam implements ObjectDetectionIO {
  private final PhotonCamera camera;
  private final Transform3d cameraToTarget;

  public ObjectDetectionIOArducam() {
    camera = new PhotonCamera(Constants.ObjectDetection.CAMERA_NAME);
    cameraToTarget = new Transform3d();
  }

  public void updateInputs(ObjectDetectionIOInputsAutoLogged inputs) {
    var instance = camera.getAllUnreadResults().get(camera.getAllUnreadResults().size() - 1);

    inputs.latestTargetsObservation = new TargetObservation(instance.getTargets());
    inputs.latestGroupedTargets = getGroups(inputs);
    inputs.isCameraConnected = camera.isConnected();
    inputs.seesFuel = instance.hasTargets();
    inputs.fuelCount = instance.getTargets().size();
  }

  /* Get Pipeline Targets & Group Them */
  private TargetGroups getGroups(ObjectDetectionIOInputsAutoLogged inputs) {
    List<List<PhotonTrackedTarget>> fuelGroups = new ArrayList<>(); // new group of groups
    List<PhotonTrackedTarget> targets =
        inputs.latestTargetsObservation.targets(); // new list of targets
    for (PhotonTrackedTarget target : targets) {
      if (fuelGroups.isEmpty()) {
        List<PhotonTrackedTarget> group = new ArrayList<>();
        group.add(
            targets.get(
                0)); // If list of groups is empty, make a group of the first target and add it
      } else { // else, iterate through all groups and compare target to all other targets in groups
        // and find the smallest distance
        // checks for case -1, which results in needing the creation of a new group (fuel isn't
        // close to any previously compared fuel)
        switch (getClosestGroupIndex(target, fuelGroups)) {
          case -1:
            List<PhotonTrackedTarget> group = new ArrayList<>();
            group.add(target);
            fuelGroups.add(group);
            break;
          default:
            fuelGroups.get(getClosestGroupIndex(target, fuelGroups)).add(target);
            break;
        }
      }
    }
    return new TargetGroups(fuelGroups);
  }

  // Finds the Index of the closest group compared to a fuel
  private int getClosestGroupIndex(
      PhotonTrackedTarget target, List<List<PhotonTrackedTarget>> groups) {
    double lowestDistance =
        Double
            .POSITIVE_INFINITY; // arbitrary large number so no matter what the first lowestDistance
    // comparison is always true
    int closestGroupIndex =
        -1; // in case if statement doesn't trigger (aka not within distance threshold or a lower
    // target distance), returns a known number
    for (int i = 0; i < groups.size(); i++) {
      for (PhotonTrackedTarget comparisonFuel : groups.get(i)) {
        double targetDistance =
            targetToTargetDistance(target.bestCameraToTarget, comparisonFuel.bestCameraToTarget);
        if (targetDistance <= Constants.ObjectDetection.DISTANCE_THRESHHOLD_INCHES
            && lowestDistance > targetDistance) {
          lowestDistance = targetDistance;
          closestGroupIndex = i;
        }
      }
    }
    return closestGroupIndex;
  }

  // Finds distance between 2 targets
  private double targetToTargetDistance(Transform3d target, Transform3d comparison) {
    // gets forward x and right y to get & puts them in distance formula
    return Math.sqrt(
        (target.getX() - comparison.getX()) * (target.getX() - comparison.getX())
            + (target.getY() - comparison.getY()) * (target.getY() - comparison.getY()));
  }
}
