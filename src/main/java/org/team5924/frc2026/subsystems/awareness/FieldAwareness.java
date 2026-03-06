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

  private Translation2d[] getOffsetsToTrenches(){
    Translation2d[] offsets = new Translation2d[4];
    Translation2d robotPosition = RobotState.getInstance().getOdometryPose().getTranslation();

    for (int i = 0; i < 4; i++) {
      offsets[i] = Constants.Field.TRENCH_TRANSLATIONS[i].minus(robotPosition);
    }

    return offsets;
  }

  private void tdfyguijoi() { // TODO: name later
    ChassisSpeeds speeds = RobotState.getInstance().getRobotChassisSpeeds();
    Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      
    }
  }
}
