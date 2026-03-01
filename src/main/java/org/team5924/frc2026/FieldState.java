package org.team5924.frc2026;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.Getter;

public class FieldState {
  private static FieldState instance;

  public static FieldState getInstance() {
    if (instance == null) instance = new FieldState();
    return instance;
  }

  /** returns the current match time in seconds */
  public double getTime() {
    return ((double) Logger.getTimestamp() * 1.0E-6);
  }

  public enum MatchShift {
    AUTO,
    TRANSITION,
    SHIFT_ONE,
    SHIFT_TWO,
    SHIFT_THREE,
    SHIFT_FOUR,
    END_GAME,

    NONE, // default state on startup
    INVALID // use this state if any info from FMS is invalid
  }

  private final double[] matchShiftTimes = {
    9999, // auto is not during teleop, so arbitrarily high time!!
    130,
    105,
    80,
    55,
    30,
    0
  };

  @Getter private MatchShift currentMatchShift = MatchShift.NONE;

  private MatchShift calculateCurrentMatchShift() {
    double time = getTime();

    if (DriverStation.isAutonomousEnabled()) {
      return MatchShift.AUTO;
    }


    return MatchShift.INVALID;
  }

  public void updateCurrentMatchShift() {
    Logger.recordMetadata("FieldState/MatchShift", (currentMatchShift = calculateCurrentMatchShift()).toString());
  }

  public boolean isHubActive() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    Alliance alliance = optionalAlliance.isPresent() ? optionalAlliance.get() : Alliance.Blue;

    

    switch (currentMatchShift) {
      case AUTO, TRANSITION, END_GAME -> {return true;}
      default -> {return false;}
    }
  }
}
