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

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2026.util.Elastic;
import org.team5924.frc2026.util.Elastic.Notification;
import org.team5924.frc2026.util.Elastic.Notification.NotificationLevel;

public class ObjectDetection extends SubsystemBase {

  private final ObjectDetectionIO io;
  private final Alert cameraDisconnected;
  private final Notification cameraDisconnectedNotification;
  private ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();

  public ObjectDetection(ObjectDetectionIO io) {
    this.io = io;
    cameraDisconnected = new Alert("Object Detection Camera Disconnected!", AlertType.kWarning);
    cameraDisconnectedNotification =
        new Notification(NotificationLevel.WARNING, "Object Detection Camera Disconnected!", "");

    CommandScheduler.getInstance().unregisterSubsystem(this);

    Notifier notifier = new Notifier(() -> periodic());
    notifier.startPeriodic(0.1);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Object Detection Inputs", inputs);
    int groupID = 0;
    for (TargetGroup group : inputs.latestGroupedTargets.groups()) {
      group.logGroup(groupID);

      groupID++;
    }

    if (!inputs.isCameraConnected) {
      Elastic.sendNotification(cameraDisconnectedNotification);
    }

    cameraDisconnected.set(!inputs.isCameraConnected);
  }

  public PhotonTrackedTarget getFirstTargetOfLargestGroup() {
    TargetGroup largestGroup = new TargetGroup(0);
    for (var group : inputs.latestGroupedTargets.groups()) {
      if (largestGroup.fuelAmount < group.fuelAmount) {
        largestGroup = group;
      }
    }
    return largestGroup.firstFiducialTarget.fuel;
  }
}
