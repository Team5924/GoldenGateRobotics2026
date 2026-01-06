/*
 * Vision.java
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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2026.Constants;

public class Vision extends SubsystemBase implements VisionIO {
  private final VisionIOCameraInputsAutoLogged inputs[] = new VisionIOCameraInputsAutoLogged[4];

  /**
   * Camera class inheriting from PhotonCamera that contain its offset relative to the robot and a
   * poseEstimator
   */
  public static class Camera extends PhotonCamera {
    public final Transform3d robotToCamera;
    public final PhotonPoseEstimator poseEstimator;

    public Camera(String initialName, Transform3d robotToCamera) {
      super(initialName);
      this.robotToCamera = robotToCamera;
      poseEstimator =
          new PhotonPoseEstimator(
              AprilTagFieldLayout.loadField(Constants.Field.FIELD_TYPE),
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              robotToCamera);
    }
  }

  private final Camera[] cameras;

  public Vision() {
    // initialize all the cameras
    cameras =
        new Camera[] {
          new Camera(Constants.Vision.FRONT_RIGHT_NAME, Constants.Vision.FRONT_RIGHT_TRANSFORM),
          new Camera(Constants.Vision.FRONT_LEFT_NAME, Constants.Vision.FRONT_LEFT_TRANSFORM),
          new Camera(Constants.Vision.BACK_RIGHT_NAME, Constants.Vision.BACK_RIGHT_TRANSFORM),
          new Camera(Constants.Vision.BACK_LEFT_NAME, Constants.Vision.BACK_LEFT_TRANSFORM)
        };

    for (int i = 0; i < inputs.length; ++i) {
      inputs[i] = new VisionIOCameraInputsAutoLogged();
    }
  }

  // TODO: add camera disconnected alerts sometime later

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateInputs(inputs);
    for (int i = 0; i < inputs.length; ++i) Logger.processInputs("Vision - Camera " + i, inputs[i]);
  }

  /** Updates AdvantageKit inputs */
  @Override
  public void updateInputs(VisionIOCameraInputs[] inputsArray) {
    for (int i = 0; i < inputsArray.length; ++i) {
      // References to the camera and its corresponding inputs
      Camera camera = cameras[i];
      VisionIOCameraInputs inputs = inputsArray[i];

      // Update the inputs
      inputs.isConnected = camera.isConnected();
      inputs.robotToCamera = camera.robotToCamera;
    }
  }

  /**
   * Updates AdvantageKit inputs for a camera index given a PhotonPipeline result
   *
   * @param results the results from the camera
   * @param cameraIndex the index of the camera
   */
  private void updateInputsWithResult(PhotonPipelineResult results, int cameraIndex) {
    VisionIOCameraInputs in = inputs[cameraIndex];

    in.targetCount = results.getTargets().size();

    if (in.targetCount == 0) {
      // no targets; set the values relating to targets to -1 to avoid confusion
      in.bestTargetPoseAmbiguity = -1;
      in.bestTargetArea = -1;
      return;
    }

    PhotonTrackedTarget bestTarget = results.getBestTarget();
    in.bestTargetPoseAmbiguity = bestTarget.poseAmbiguity;
    in.bestTargetArea = bestTarget.area;
  }

  // ----- Vision calculation stuff here and below -----

  /** Periodically adds measurements from each camera to the estimator */
  public void periodicAddMeasurements(SwerveDrivePoseEstimator estimator) {
    for (int i = 0; i < cameras.length; ++i) {
      Camera camera = cameras[i];
      ArrayList<PhotonPipelineResult> results = filterResults(camera.getAllUnreadResults());

      // update the inputs with the most recent result
      if (!results.isEmpty()) {
        PhotonPipelineResult mostRecentResult = results.get(results.size() - 1);
        updateInputsWithResult(mostRecentResult, i);
      }

      // get the estimated poses, loop through them, and include each good estimate in the vision
      // measurement calculation
      ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(camera, results);
      for (EstimatedRobotPose pose : estimatedPoses) {
        if (isInsideField(pose.estimatedPose.getTranslation().toTranslation2d())
            && (pose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                || (pose.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                    && pose.targetsUsed.get(0).poseAmbiguity < 0.05
                    && pose.targetsUsed.get(0).area > 0.25))) {
          estimator.addVisionMeasurement(
              pose.estimatedPose.toPose2d(), pose.timestampSeconds); // don't need stds
        }
      }
    }
  }

  /** Returns whether or not all the cameras are connected */
  public boolean allCamerasConnected() {
    for (Camera camera : cameras) {
      if (!camera.isConnected()) return false;
    }
    return true;
  }

  /** Filters out results from each camera that include barge positions */
  private ArrayList<PhotonPipelineResult> filterResults(List<PhotonPipelineResult> results) {
    ArrayList<PhotonPipelineResult> updatedResults = new ArrayList<PhotonPipelineResult>(results);
    for (int i = results.size() - 1; i >= 0; --i) {
      if (removeResult(results.get(i))) updatedResults.remove(i);
    }
    return updatedResults;
  }

  /** Returns whether or not to remove the result (which is if it uses barge aprilTags) */
  private boolean removeResult(PhotonPipelineResult result) {
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (Constants.Vision.BARGE_TAG_IDS.contains(target.getFiducialId())) return true;
    }
    return false;
  }

  /** Gets and returns the estimated poses from a camera */
  private ArrayList<EstimatedRobotPose> getEstimatedPoses(
      Camera camera, ArrayList<PhotonPipelineResult> results) {
    ArrayList<EstimatedRobotPose> poses = new ArrayList<EstimatedRobotPose>();
    for (PhotonPipelineResult result : results) {
      Optional<EstimatedRobotPose> pose = camera.poseEstimator.update(result);
      if (pose.isPresent()) poses.add(pose.get());
    }

    return poses;
  }

  /** Returns whether or not a position is within the field */
  private boolean isInsideField(Translation2d position) {
    return position.getX() >= -Constants.Field.FIELD_BORDER_MARGIN
        && position.getX() <= Constants.Field.FIELD_LENGTH + Constants.Field.FIELD_BORDER_MARGIN
        && position.getY() >= -Constants.Field.FIELD_BORDER_MARGIN
        && position.getY() <= Constants.Field.FIELD_WIDTH + Constants.Field.FIELD_BORDER_MARGIN;
  }
}
