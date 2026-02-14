/*
 * VisionConstants.java
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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.simulation.SimCameraProperties;
import org.team5924.frc2026.util.VisionUtil;

public class VisionConstants {
  public static final String FRONT_RIGHT_NAME = "Front Right";
  public static final String FRONT_LEFT_NAME = "Front Left";
  public static final String BACK_RIGHT_NAME = "Back Right";
  public static final String BACK_LEFT_NAME = "Back Left";

  // TODO: update these transforms!!
  public static final double CAMERA_CENTER_DISTANCE = Units.inchesToMeters(11.8645);
  public static final double CAMERA_Z = Units.inchesToMeters(7.0);
  public static final double CAMERA_PITCH = -Math.toRadians(45.0);

// +x = forward, +y = left, +z = up
  public static final Transform3d FRONT_LEFT_TRANSFORM =
      new Transform3d(
          new Translation3d(CAMERA_CENTER_DISTANCE, CAMERA_CENTER_DISTANCE, CAMERA_Z),
          new Rotation3d(0.0, CAMERA_PITCH, Math.PI / 4));
  public static final Transform3d FRONT_RIGHT_TRANSFORM =
      new Transform3d(
          new Translation3d(CAMERA_CENTER_DISTANCE, -CAMERA_CENTER_DISTANCE, CAMERA_Z),
          new Rotation3d(0.0, CAMERA_PITCH, -Math.PI / 4));
  public static final Transform3d BACK_LEFT_TRANSFORM =
      new Transform3d(
          new Translation3d(-CAMERA_CENTER_DISTANCE, CAMERA_CENTER_DISTANCE, CAMERA_Z),
          new Rotation3d(0.0, CAMERA_PITCH, 3 * Math.PI / 4));
  public static final Transform3d BACK_RIGHT_TRANSFORM =
      new Transform3d(
          new Translation3d(-CAMERA_CENTER_DISTANCE, -CAMERA_CENTER_DISTANCE, CAMERA_Z),
          new Rotation3d(0.0, CAMERA_PITCH, -3 * Math.PI / 4));

  // public static final ArrayList<Integer> IGNORE_IDS =
  //     new ArrayList<Integer>(List.of());

  // https://www.arducam.com/100fps-global-shutter-color-usb-camera-board-1mp-ov9782-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-for-computer-laptop-android-device-and-raspberry-pi-arducam.html
  public static final SimCameraProperties SIM_ARDUCAM_PROPERIES =
      new SimCameraProperties()
          .setCalibration(1280, 800, VisionUtil.getDiagFov(1280, 800, 70))
          .setFPS(20)
          .setCalibError(0, 0) // TODO: update values below here
          .setAvgLatencyMs(0)
          .setLatencyStdDevMs(0);

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // FL
        1.0, // FR
        1.0, // BL
        1.0 // BR
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
