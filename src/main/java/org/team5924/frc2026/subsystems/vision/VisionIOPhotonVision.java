/*
 * VisionIOPhotonVision.java
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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import lombok.Setter;
import org.photonvision.PhotonCamera;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.util.LoggedTunableNumber;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  @Setter protected Transform3d robotToCamera;

  private final LoggedTunableNumber rollDegrees;
  private final LoggedTunableNumber pitchDegrees;
  private final LoggedTunableNumber yawDegrees;

  private final LoggedTunableNumber xInches;
  private final LoggedTunableNumber yInches;
  private final LoggedTunableNumber zInches;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    // this.robotToCamera = robotToCamera;

    rollDegrees =
        new LoggedTunableNumber(
            "Vision/Camera" + camera.getName() + "/rotation roll degrees",
            Units.radiansToDegrees(robotToCamera.getRotation().getX()));
    pitchDegrees =
        new LoggedTunableNumber(
            "Vision/Camera" + camera.getName() + "/rotation pitch degrees",
            Units.radiansToDegrees(robotToCamera.getRotation().getY()));
    yawDegrees =
        new LoggedTunableNumber(
            "Vision/Camera" + camera.getName() + "/rotation yaw degrees",
            Units.radiansToDegrees(robotToCamera.getRotation().getZ()));

    xInches =
        new LoggedTunableNumber(
            "Vision/Camera" + camera.getName() + "/position x inches",
            Units.metersToInches(robotToCamera.getX()));
    yInches =
        new LoggedTunableNumber(
            "Vision/Camera" + camera.getName() + "/position y inches",
            Units.metersToInches(robotToCamera.getY()));
    zInches =
        new LoggedTunableNumber(
            "Vision/Camera" + camera.getName() + "/position z inches",
            Units.metersToInches(robotToCamera.getZ()));

    this.robotToCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(xInches.getAsDouble()),
                Units.inchesToMeters(yInches.getAsDouble()),
                Units.inchesToMeters(zInches.getAsDouble())),
            new Rotation3d(
                Units.degreesToRadians(rollDegrees.getAsDouble()),
                Units.degreesToRadians(pitchDegrees.getAsDouble()),
                Units.degreesToRadians(yawDegrees.getAsDouble())));
  }

  private void updateLoggedTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            robotToCamera =
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(xInches.getAsDouble()),
                        Units.inchesToMeters(yInches.getAsDouble()),
                        Units.inchesToMeters(zInches.getAsDouble())),
                    new Rotation3d(
                        Units.degreesToRadians(rollDegrees.getAsDouble()),
                        Units.degreesToRadians(pitchDegrees.getAsDouble()),
                        Units.degreesToRadians(yawDegrees.getAsDouble()))),
        xInches,
        yInches,
        zInches,
        rollDegrees,
        pitchDegrees,
        yawDegrees);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    updateLoggedTunableNumbers();
    inputs.connected = camera.isConnected();
    inputs.cameraToTarget = null;

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = Constants.Field.field.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;

          Rotation3d cameraToTargetRotation = cameraToTarget.getRotation();
          inputs.cameraToTarget =
              new TranslationRotation(
                  cameraToTarget.getTranslation(),
                  Units.radiansToDegrees(cameraToTargetRotation.getX()),
                  Units.radiansToDegrees(cameraToTargetRotation.getY()),
                  Units.radiansToDegrees(cameraToTargetRotation.getZ()));

          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
