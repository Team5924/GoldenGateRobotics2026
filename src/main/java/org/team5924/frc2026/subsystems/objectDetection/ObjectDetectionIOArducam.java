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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2026.Constants;

public class ObjectDetectionIOArducam implements ObjectDetectionIO {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera; // the position of the Camera relative to the robot
  private final Timer timer = new Timer();

  public ObjectDetectionIOArducam() {
    camera = new PhotonCamera(Constants.ObjectDetection.CAMERA_NAME);
    robotToCamera = new Transform3d();
    timer.start();
  }

  public void updateInputs(ObjectDetectionIOInputsAutoLogged inputs) {
    inputs.cameraName = camera.getName();
    inputs.isCameraConnected = camera.isConnected();

    var results = camera.getAllUnreadResults();

    inputs.resultsSize = results.size();

    if (results.size() == 0) {
      return;
    }

    var instance = results.get(results.size() - 1);
    inputs.latestTargetsObservation = new TargetObservation(instance.getTargets());
    inputs.latestGroupedTargets = getGroups(instance.getTargets());
    inputs.seesFuel = instance.hasTargets();
    inputs.fuelCount = instance.getTargets().size();
    inputs.groupCount = inputs.latestGroupedTargets.groups().size();
  }

  /* Get Pipeline Targets & Group Them */
  private TargetGroups getGroups(List<PhotonTrackedTarget> targets) {
    List<TargetGroup> fuelGroups = new ArrayList<>(); // new group of groups
    for (PhotonTrackedTarget target : targets) {
      if (fuelGroups.isEmpty()) {
        TargetGroup group = new TargetGroup();
        group.addTarget(
            target); // If list of groups is empty, make a group of the first target and add it
        fuelGroups.add(group);
      } else { // else, iterate through all groups and compare target to all other targets in groups
        // and find the smallest distance
        // checks for case -1, which results in needing the creation of a new group (fuel isn't
        // close to any previously compared fuel)
        int closestGroupIndex = getClosestGroupIndex(target, fuelGroups);

        switch (closestGroupIndex) {
          case -1:
            TargetGroup group = new TargetGroup();
            group.addTarget(target);
            fuelGroups.add(group);
            break;
          default:
            fuelGroups.get(closestGroupIndex).addTarget(target);
            break;
        }
      }
    }

    if (timer.get() >= 0.5) {
      logGroups(fuelGroups);
      timer.reset();
    }

    return new TargetGroups(fuelGroups);
  }

  private void logGroups(List<TargetGroup> fuelGroups) {
    for (int i = 0; i < fuelGroups.size(); ++i) {
      List<PhotonTrackedTarget> targets = fuelGroups.get(i).targets;
      System.out.println("-------- GROUP " + i + " --------> " + targets.size());
      for (int j = 0; j < targets.size(); ++j) {
        PhotonTrackedTarget target = targets.get(j);
        System.out.println(
            "  TARGET "
                + j
                + "  --  Position: "
                + target.getBestCameraToTarget());
      }
    }
  }

  // Finds the Index of the closest group compared to a fuel
  private int getClosestGroupIndex(PhotonTrackedTarget target, List<TargetGroup> groups) {
    double lowestDistance =
        Double
            .POSITIVE_INFINITY; // arbitrary large number so no matter what the first lowestDistance
    // comparison is always true
    int closestGroupIndex =
        -1; // in case if statement doesn't trigger (aka not within distance threshold or a lower
    // target distance), returns a known number
    for (int i = 0; i < groups.size(); i++) {
      for (PhotonTrackedTarget comparisonFuel : groups.get(i).targets) {
        double targetDistance =
            Units.metersToInches(
                targetToTargetDistance(
                    target.bestCameraToTarget, comparisonFuel.bestCameraToTarget));

        // if (timer.get() >= 0.5) {
          // logComparison(target, comparisonFuel, groups, i, targetDistance);
        // }
        if (targetDistance <= Constants.ObjectDetection.DISTANCE_THRESHHOLD_INCHES
            && lowestDistance > targetDistance) {
          lowestDistance = targetDistance;
          closestGroupIndex = i;
        }
      }
    }
    return closestGroupIndex;
  }

  private void logComparison(PhotonTrackedTarget target, PhotonTrackedTarget comparisonFuel, List<TargetGroup> groups, int i, double targetDistance) {
          System.out.println("i: " + i + " // pitch: " + comparisonFuel.pitch);
          System.out.println(
              "TARGET SIZE THING:: :: : : : :: : :  " + groups.get(i).targets.size() + "\n");
          System.out.println(
              "-------TARGET DISTANCE-------\n"
                  + "Target "
                  + target.getDetectedObjectClassID()
                  + " to Target "
                  + comparisonFuel.getDetectedObjectClassID()
                  + ": "
                  + targetDistance);
  }

  // Finds distance between 2 targets
  private double targetToTargetDistance(Transform3d target, Transform3d comparison) {
    // gets forward x and right y to get & puts them in distance formula
    return Units.metersToInches(target.getTranslation().getDistance(comparison.getTranslation()));
  }
}
