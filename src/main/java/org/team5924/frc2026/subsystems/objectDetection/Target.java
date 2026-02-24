/*
 * Target.java
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

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.io.Serializable;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Target implements Serializable {
  public int fuelID;
  public double distanceToRobotFeet;
  public PhotonTrackedTarget fuel;
  public Transform2d robotToFuel;
  public Transform2d robotToFuelFeet;

  public Target() {
    fuelID = 0;
    distanceToRobotFeet = 0.0;
    fuel = new PhotonTrackedTarget();
    robotToFuel = new Transform2d();
  }

  public Target(
      int fuelID, double distanceToRobot, PhotonTrackedTarget fuel, Transform2d robotToFuel) {
    this.fuelID = fuelID;
    this.distanceToRobotFeet = distanceToRobot;
    this.fuel = fuel;
    this.robotToFuel = robotToFuel;
    robotToFuelFeet =
        new Transform2d(
            Units.metersToFeet(robotToFuel.getX()),
            Units.metersToFeet(robotToFuel.getY()),
            robotToFuel.getRotation());
  }

  public Translation2d getRobotOffset() {
    return new Translation2d(robotToFuel.getX(), robotToFuel.getX());
  }

  public void logTarget(String logPath) {
    Logger.recordOutput(logPath + "/distanceFeet", distanceToRobotFeet);
    Logger.recordOutput(logPath + "/fuelID", fuelID);
    Logger.recordOutput(logPath + "/robotToFuelTransform2d", robotToFuel);
  }

  @Override
  public String toString() {
    return "Target ID: " + fuelID + "\nDistance to Robot in Feet: " + distanceToRobotFeet;
  }
}
