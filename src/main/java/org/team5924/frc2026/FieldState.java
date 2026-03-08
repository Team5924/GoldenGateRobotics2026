/*
 * FieldState.java
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

package org.team5924.frc2026;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

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
    130, 105, 80, 55, 30, 0
  };

  @Getter private MatchShift currentMatchShift = MatchShift.NONE;

  private Alliance autoWinner;
  private boolean hasAutoWinner;

  private MatchShift calculateCurrentMatchShift() {
    double time = getTime();

    if (DriverStation.isAutonomousEnabled()) {
      return MatchShift.AUTO;
    }

    return MatchShift.INVALID;
  }

  public void updateCurrentMatchShift() {
    Logger.recordMetadata(
        "FieldState/MatchShift", (currentMatchShift = calculateCurrentMatchShift()).toString());
  }

  public boolean isHubActive() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    Alliance alliance = optionalAlliance.isPresent() ? optionalAlliance.get() : Alliance.Blue;

    switch (currentMatchShift) {
      case AUTO, TRANSITION, END_GAME -> {
        return true;
      }
      case NONE, INVALID -> {
        return true;
      } // allow shooting on error?
      case SHIFT_ONE, SHIFT_TWO -> {
        return (false);
      }
      default -> {
        return false;
      }
    }
  }
}
