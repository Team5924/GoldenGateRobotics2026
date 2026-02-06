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

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2026.Constants;

public class ObjectDetectionIOArducam implements ObjectDetectionIO {
  private final PhotonCamera camera;

  public ObjectDetectionIOArducam() {
    camera = new PhotonCamera(Constants.ObjectDetection.CAMERA_NAME);
  }

  public void updateInputs(ObjectDetectionIOInputsAutoLogged inputs) {
    var instance = camera.getAllUnreadResults().get(camera.getAllUnreadResults().size() - 1);

    inputs.latestTargetsObservation = new TargetObservation(instance.getTargets());
    inputs.latestGroupedTargets = new TargetGroups(getGroups(inputs));
    inputs.isCameraConnected = camera.isConnected();
    inputs.seesFuel = instance.hasTargets();
    inputs.fuelCount = instance.getTargets().size();
  }

  /* Get Pipeline Targets & Group Them */
  private List<List<PhotonTrackedTarget>> getGroups(ObjectDetectionIOInputsAutoLogged inputs) {
    //Initialize List of List of Targets (Groups) and the List of Targets that go in the List of List of Targets

//     List<List<PhotonTrackedTarget>> results = new ArrayList<>();
//     List<PhotonTrackedTarget> result = new ArrayList<>();
//     result.add(inputs.latestTargetsObservation.targets().get(0));

//     for (int i = 1; i < inputs.latestTargetsObservation.targets().size(); i++) {
//       PhotonTrackedTarget target = inputs.latestTargetsObservation.targets().get(i);
//       PhotonTrackedTarget comparison = inputs.latestTargetsObservation.targets().get(i - 1);
//       if (Math.abs(target.bestCameraToTarget.getX() - comparison.bestCameraToTarget.getX())
//               < Constants.ObjectDetection.X_DISTANCE_THRESHHOLD_INCHES
//           && Math.abs(target.bestCameraToTarget.getY() - comparison.bestCameraToTarget.getY())
//               < Constants.ObjectDetection.Y_DISTANCE_THRESHHOLD_INCHES) {
//         result.add(target);
//       } else {
//         results.add(result);
//         result = new ArrayList<>();
//         result.add(inputs.latestTargetsObservation.targets().get(i));
//       }
//     }
//     return results;
//   }

    //Initialize List of Targets(fuelGroups) and ungrouped List
    List<List<PhotonTrackedTarget>> fuelGroups = new ArrayList<>();
    List<PhotonTrackedTarget> targets = inputs.latestTargetsObservation.targets();
    for (var target : targets) {
        List<PhotonTrackedTarget> group = new ArrayList<>();
        if (fuelGroups.isEmpty()) {
                group.add(targets.get(0));
        } else {
            double lowestDistance ;
            int groupIndex;
            for (int i = 0; i < fuelGroups.size(); i++) {
                for (var comparison : fuelGroups.get(i)) {
                    double xDistance = Math.abs(comparison.bestCameraToTarget.getX() - target.bestCameraToTarget.getX());
                    double yDistance = Math.abs(comparison.bestCameraToTarget.getY() - target.bestCameraToTarget.getY());

                    if ((xDistance <= Constants.ObjectDetection.X_DISTANCE_THRESHHOLD_INCHES 
                            && yDistance <= Constants.ObjectDetection.Y_DISTANCE_THRESHHOLD_INCHES)
                        && lowestDistance < Math.sqrt(xDistance*xDistance + yDistance*yDistance)) {
                        lowestDistance = Math.sqrt(xDistance*xDistance + yDistance*yDistance);
                        groupIndex = i;
                    }
                }
            }
            fuelGroups.get(groupIndex).add(target);
        }
    }

}
