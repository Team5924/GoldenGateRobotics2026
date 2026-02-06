/*
 * ObjectDetection.java
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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {

  private final ObjectDetectionIO io;
  private final Alert cameraDisconnected;
  private ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();


  public ObjectDetection(ObjectDetectionIO io) {
    this.io = io;
    cameraDisconnected = new Alert("Object Detection Camera Disconnected!", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Object Detection Inputs", inputs);

  }

  public Transform3d getVectorLargestGroup() {
    List<PhotonTrackedTarget> largestGroup = new ArrayList<>();
    for (var group : inputs.latestGroupedTargets.groups()) {
        if (largestGroup.size() < group.size()) {
            largestGroup = group;
        }
    }
    return largestGroup.get(0).bestCameraToTarget;
  }
}
