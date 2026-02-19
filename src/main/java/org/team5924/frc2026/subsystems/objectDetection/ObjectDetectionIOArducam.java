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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2026.Constants;

public class ObjectDetectionIOArducam implements ObjectDetectionIO {
  private final PhotonCamera camera;
  private final Timer timer = new Timer();

  public ObjectDetectionIOArducam() {
    camera = new PhotonCamera(Constants.ObjectDetection.CAMERA_NAME);
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
    logGroups(inputs.latestGroupedTargets.groups());
  }

  private void logGroups(List<TargetGroup> groups) {
    for (int groupIndex = 0; groupIndex < groups.size(); ++groupIndex) {
      TargetGroup group = groups.get(groupIndex);

      List<Target> targets = group.targets;
      for (int targetIndex = 0; targetIndex < targets.size(); ++targetIndex) {
        Target target = targets.get(targetIndex);

        String logPath =
            "Object Detection Inputs/Target Group " + groupIndex + "/Target " + target.fuelID + "/";
        Logger.recordOutput(
            logPath + "distanceToRobot",
            Units.metersToFeet(ObjectDetectionUtils.getRobotToTargetDistance(target.fuel)));
        Logger.recordOutput(
            logPath + "cameraToTarget",
            ObjectDetectionUtils.getRobotToTargetTranslation2d(target.fuel));
      }
    }
  }

  /* Get Pipeline Targets & Group Them */
  private TargetGroups getGroups(List<PhotonTrackedTarget> targets) {
    List<TargetGroup> fuelGroups = new ArrayList<>(); // new group of groups
    int currentID = 0;
    for (PhotonTrackedTarget target : targets) {
      Target fuelTarget =
          new Target(
              currentID,
              ObjectDetectionUtils.getRobotToTargetDistance(target),
              target,
              ObjectDetectionUtils.getRobotToTargetTransform2d(target));
      if (fuelGroups.isEmpty()) {
        TargetGroup group = new TargetGroup();
        group.addTarget(
            fuelTarget); // If list of groups is empty, make a group of the first target and add it
        fuelGroups.add(group);
      } else { // else, iterate through all groups and compare target to all other targets in groups
        // and find the smallest distance
        // checks for case -1, which results in needing the creation of a new group (fuel isn't
        // close to any previously compared fuel)
        int closestGroupIndex = ObjectDetectionUtils.getClosestGroupIndex(target, fuelGroups);

        switch (closestGroupIndex) {
          case -1:
            TargetGroup group = new TargetGroup();
            group.addTarget(fuelTarget);
            fuelGroups.add(group);
            break;
          default:
            fuelGroups.get(closestGroupIndex).addTarget(fuelTarget);
            break;
        }
      }
      currentID++;
    }

    if (timer.get() >= 0.5) {
      printGroups(fuelGroups);
      timer.reset();
    }

    return new TargetGroups(fuelGroups);
  }

  private void printGroups(List<TargetGroup> fuelGroups) {
    for (int i = 0; i < fuelGroups.size(); ++i) {
      System.out.println(fuelGroups.get(i));
      // List<Target> targets = fuelGroups.get(i).targets;
      // System.out.println(
      //     "-------- GROUP "
      //         + i
      //         + " --------> "
      //         + targets.size()
      //         + "\n  FIRST SIZE : "
      //         + fuelGroups.get(i).firstFiducialTarget.distanceToRobotFeet);
      // for (int j = 0; j < targets.size(); ++j) {
      //   PhotonTrackedTarget target = targets.get(j).fuel;
      //   System.out.println(
      //       "  TARGET "
      //           + j
      //           + "  --  "
      //           + PhotonUtils.calculateDistanceToTargetMeters(
      //               Constants.ObjectDetection.CAMERA_TO_FLOOR_HEIGHT_METERS,
      //               Constants.ObjectDetection.FUEL_TOP_TO_FLOOR_METERS,
      //               target.getPitch(),
      //               Constants.ObjectDetection.CAMERA_PITCH_RADS));
      // }
    }
  }

  private void logComparison(
      PhotonTrackedTarget target,
      PhotonTrackedTarget comparisonFuel,
      List<TargetGroup> groups,
      int i,
      double targetDistance) {
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
}
