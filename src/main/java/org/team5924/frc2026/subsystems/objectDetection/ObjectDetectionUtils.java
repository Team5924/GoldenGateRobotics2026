/*
 * ObjectDetectionUtils.java
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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team5924.frc2026.Constants;

/* Handles Math & Other Util Functions for Object Detection */
public class ObjectDetectionUtils {
  private ObjectDetectionUtils() {}

  /** Returns the distance from the camera to a PhotonTrackedTarget (in meters) */
  public static double getCameraToTargetDistance(PhotonTrackedTarget target) {
    return PhotonUtils.calculateDistanceToTargetMeters(
        Constants.ObjectDetection.FLOOR_TO_CAMERA_METERS,
        Constants.ObjectDetection.FLOOR_TO_FUEL_CENTER_METERS,
        Constants.ObjectDetection.CAMERA_PITCH_RADS,
        Units.degreesToRadians(target.getPitch()));
  }

  /** Returns the distance from the robot to a PhotonTrackedTarget (in meters) */
  public static double getRobotToTargetDistance(PhotonTrackedTarget target) {
    return Math.sqrt(
            Math.pow(getCameraToTargetDistance(target), 2.0)
                - Math.pow(Constants.ObjectDetection.FUEL_CENTER_TO_CAMERA_METERS, 2.0))
        - Constants.ObjectDetection.CAMERA_OFFSET_FROM_ROBOT_FRAME_METERS;
  }

  /**
   * Returns the Translation2d from the camera to a PhotonTrackedTarget (in meters, meters), where
   * +x is forward and +y is left
   */
  public static Translation2d getCameraToTargetTranslation2d(PhotonTrackedTarget target) {
    return PhotonUtils.estimateCameraToTargetTranslation(
        getRobotToTargetDistance(target), new Rotation2d(Units.degreesToRadians(target.getYaw())));
  }

  /**
   * Returns the Translation2d from the robot to a PhotonTrackedTarget (in meters, meters), where +x
   * is forward and +y is left
   */
  public static Translation2d getRobotToTargetTranslation2d(PhotonTrackedTarget target) {
    double targetDistance = getRobotToTargetDistance(target);
    double targetYawRads = Units.degreesToRadians(target.getYaw());
    return new Translation2d(
        targetDistance * Math.cos(targetYawRads), targetDistance * Math.sin(targetYawRads));
  }

  /** Returns the distance between 2 PhotonTrackedTargets (in inches) */
  public static double getTargetToTargetDistanceInches(
      PhotonTrackedTarget target, PhotonTrackedTarget comparison) {
    Translation2d targetTranslation2d = getRobotToTargetTranslation2d(target);
    Translation2d comparisonTranslation2d = getRobotToTargetTranslation2d(comparison);

    return Units.metersToInches(targetTranslation2d.getDistance(comparisonTranslation2d));
  }

  /** Returns the Transform2d from the robot to a target + yaw (in meters, meters, rads) */
  public static Transform2d getRobotToTargetTransform2d(PhotonTrackedTarget target) {
    return new Transform2d(
        getRobotToTargetTranslation2d(target),
        new Rotation2d(Units.degreesToRadians(target.getYaw())));
  }

  // Finds the Index of the closest group compared to a another fuel
  public static int getClosestGroupIndex(Target target, List<TargetGroup> groups) {
    double lowestDistance =
        Double
            .POSITIVE_INFINITY; // arbitrary large number so no matter what the first lowestDistance
    // comparison is always true
    int closestGroupIndex =
        -1; // in case if statement doesn't trigger (aka not within distance threshold or a lower
    // target distance), returns a known number
    for (int i = 0; i < groups.size(); i++) {
      for (Target comparisonFuel : groups.get(i).targets) {
        double targetDistance = getTargetToTargetDistanceInches(target.fuel, comparisonFuel.fuel);

        // if (timer.get() >= 0.5) {
        // logComparison(target, comparisonFuel, groups, i, targetDistance);
        // }
        if (targetDistance <= Constants.ObjectDetection.FUEL_CENTER_DISTANCE_THRESHOLD_INCHES
            && lowestDistance > targetDistance) {
          lowestDistance = targetDistance;
          closestGroupIndex = i;
        }
      }
    }
    return closestGroupIndex;
  }

  // Finds the Index of the closest group compared to a another fuel
  public static int optimizedGetClosestGroupIndex(Target target, List<TargetGroup> groups) {
    double lowestDistance = Double.POSITIVE_INFINITY;
    int closestGroupIndex = -1;

    Translation2d targetPosition = target.getRobotOffset();

    for (int i = 0; i < groups.size(); i++) {
      List<Translation2d> comparisonPositions = groups.get(i).getFuelPositions();
      for (Translation2d comparisonPosition : comparisonPositions) {
        double targetDistance = targetPosition.getDistance(comparisonPosition);

        if (targetDistance <= Constants.ObjectDetection.FUEL_CENTER_DISTANCE_THRESHOLD_INCHES
            && lowestDistance > targetDistance) {
          lowestDistance = targetDistance;
          closestGroupIndex = i;
        }
      }
    }
    return closestGroupIndex;
  }
}
