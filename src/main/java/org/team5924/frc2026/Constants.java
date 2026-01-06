/*
 * Constants.java
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

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean TUNING_MODE = false;
  public static final boolean ALLOW_ASSERTS = false;
  public static final double SLOW_MODE_MULTI = 0.5;

  //   /* Field */
  //   public static final double FIELD_BORDER_MARGIN = 0.5;
  //   public static final AprilTagFieldLayout field =
  //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  //   public static final double FIELD_WIDTH = field.getFieldWidth();
  //   public static final double FIELD_LENGTH = field.getFieldLength();
  //   public static final double CORAL_STATION_RADIANS_NORMAL = 0.959931;


  /* General */
  public static final double LOOP_PERIODIC_SECONDS = 0.02;

  public final class Example {
    public static final int CAN_ID = 0;
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0;
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(35)
            .withStatorCurrentLimit(35))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

    public static final CANdiConfiguration CANDI_CONFIG =
      new CANdiConfiguration()
        .withDigitalInputs(
          new DigitalInputsConfigs()
            .withS1CloseState(S1CloseStateValue.CloseWhenLow)
            .withS2CloseState(S2CloseStateValue.CloseWhenLow));

    public static final OpenLoopRampsConfigs OPEN_LOOP_RAMPS_CONFIGS =
      new OpenLoopRampsConfigs()
        .withDutyCycleOpenLoopRampPeriod(0.02)
        .withTorqueOpenLoopRampPeriod(0.02)
        .withVoltageOpenLoopRampPeriod(0.02);

    public static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMPS_CONFIGS =
      new ClosedLoopRampsConfigs()
        .withDutyCycleClosedLoopRampPeriod(0.02)
        .withTorqueClosedLoopRampPeriod(0.02)
        .withVoltageClosedLoopRampPeriod(0.02);
  }

  public final class Vision {
    public static final String FRONT_RIGHT_NAME = "Front Right";
    public static final String FRONT_LEFT_NAME = "Front Left";
    public static final String BACK_RIGHT_NAME = "Back Right";
    public static final String BACK_LEFT_NAME = "Back Left";

    public static final Transform3d FRONT_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.012552, -0.319809, 0.191168),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-70.0)));
    public static final Transform3d BACK_RIGHT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.081165, -0.322330, 0.191168),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-(180.0 - 55.0))));
    public static final Transform3d FRONT_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.012552, 0.319809, 0.191168),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(70.0)));
    public static final Transform3d BACK_LEFT_TRANSFORM =
        new Transform3d(
            new Translation3d(-0.081165, 0.322330, 0.191168),
            new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(180.0 - 55.0)));
    public static final ArrayList<Integer> BARGE_TAG_IDS =
        new ArrayList<Integer>(Arrays.asList(4, 5, 14, 15));
  }

  public final class GenericRollerSystem {
    public static final double REDUCTION = 1.0;
    public static final double SIM_MOI = 0.001;
  }

  public final class ExampleRoller {
    public static final int CAN_ID = 0;
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0;
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(35)
            .withStatorCurrentLimit(35))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
  }

  public static final class Field { // TODO: update all of these when new field released
      public static final double FIELD_BORDER_MARGIN = 0.5;
      public static final AprilTagFields FIELD_TYPE = AprilTagFields.k2025Reefscape; // TODO: update to new field
      public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(FIELD_TYPE);
      public static final double FIELD_WIDTH = field.getFieldWidth();
      public static final double FIELD_LENGTH = field.getFieldLength();

      public static final double faceLength = Units.inchesToMeters(36.792600);
      public static final double fieldWidth = field.getFieldWidth();
      public static final Translation2d blueCenter =
          new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.5));

      public static final Translation2d redCenter =
          new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
  }
}
