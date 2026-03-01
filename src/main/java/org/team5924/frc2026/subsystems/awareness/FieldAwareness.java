package org.team5924.frc2026.subsystems.awareness;

import org.team5924.frc2026.Constants;
import org.team5924.frc2026.RobotState;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FieldAwareness {
  private static FieldAwareness instance;


  public static FieldAwareness getInstance() {
    if (instance == null) instance = new FieldAwareness();
    return instance;
  }

  private double[] getDistanceToTrenches(){
    double[] distance = new double[4];
    Translation2d robotPosition = RobotState.getInstance().getOdometryPose().getTranslation();

    for (int i = 0; i < 4; i++) {
      distance[i] = Constants.Field.TRENCH_TRANSLATIONS[i].getDistance(robotPosition);
    }

    return distance;
  }

  private void tdfyguijoi() { // TOD: name later
    ChassisSpeeds speeds = RobotState.getInstance().getRobotChassisSpeeds();
    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);


  }
}
