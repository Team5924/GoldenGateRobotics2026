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

import org.team5924.frc2026.subsystems.pivots.intakePivot.IntakePivot.IntakePivotState;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double TRACK_WIDTH_Y_METERS = Units.inchesToMeters(19.5);
  public static final boolean TUNING_MODE = false; // TODO: tuning mode off

  public static final boolean ALLOW_ASSERTS = false;
  public static final double SLOW_MODE_MULTI = 0.33;

  public static final double OVERHEAT_THRESHOLD = 85.0;

  /* General Robot */
  public static final double LOOP_PERIODIC_SECONDS = 0.02;
  public static final double SYS_ID_TIME = 10.0;
  public static final double JOYSTICK_DEADZONE = 0.05;

  public final class Field {
    // origin = rightmost corner of blue alliance wall
    // +x -> towards red alliance wall

    public static final double BORDER_MARGIN = 0.5;
    public static final AprilTagFieldLayout field =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double LENGTH = field.getFieldLength(); // x
    public static final double WIDTH = field.getFieldWidth(); // y

    public static final double CENTER_X = WIDTH / 2;
    public static final double CENTER_Y = LENGTH / 2;
    public static final double HUB_X = CENTER_X - 143.50;
  }

  /* ### Subystems ### */

  /* Subsystem Configs */
  public static final OpenLoopRampsConfigs GENERIC_OPEN_LOOP_RAMPS_CONFIGS =
    new OpenLoopRampsConfigs()
      .withDutyCycleOpenLoopRampPeriod(0.02)
      .withTorqueOpenLoopRampPeriod(0.02)
      .withVoltageOpenLoopRampPeriod(0.02);

  public static final ClosedLoopRampsConfigs GENERIC_CLOSED_LOOP_RAMPS_CONFIGS =
    new ClosedLoopRampsConfigs()
      .withDutyCycleClosedLoopRampPeriod(0.02)
      .withTorqueClosedLoopRampPeriod(0.02)
      .withVoltageClosedLoopRampPeriod(0.02);

  public final class GenericRoller {
    public static final TalonFXConfiguration CLOCKWISE_CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

    public static final TalonFXConfiguration COUNTERCLOCKWISE_CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

    public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withRotorToSensorRatio(1.0);
  }

  public final class Intake {
    public static final int CAN_ID = 41;
    public static final int FOLLOWER_CAN_ID = 42;
    public static final String BUS = "rio";
    public static final double MOTOR_TO_MECHANISM = 36.0 / 16.0;
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration CONFIG = GenericRoller.COUNTERCLOCKWISE_CONFIG.clone();
  }

  public final class IntakePivot {
    public static final int CAN_ID = 40;
    public static final String BUS = "rio";
    public static final double SIM_MOI = 0.001;

    public static final double MOTOR_TO_MECHANISM =
      (4.0 / 1.0) * (4.0 / 1.0) * (54.0 / 20.0) * (32.0 / 12.0);

    public static final double EPSILON_RADS = Units.degreesToRadians(5.0); // 0.035 rads
    public static final double STATE_TIMEOUT = 5.0;
    public static final boolean ENABLE_TIMEOUT = false;

    /** how far the intake pivot physically rotates */
    public static final double MIN_POSITION_ROTATIONS =
      Units.radiansToRotations(IntakePivotState.DOWN.getRads().getAsDouble());
    public static final double MAX_POSITION_ROTATIONS =
      Units.radiansToRotations(IntakePivotState.STOW.getRads().getAsDouble());

    public static final double MIN_POSITION_RADS = Units.rotationsToRadians(MIN_POSITION_ROTATIONS);
    public static final double MAX_POSITION_RADS = Units.rotationsToRadians(MAX_POSITION_ROTATIONS);

    /* Configs */
    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

    public static final SoftwareLimitSwitchConfigs SOFTWARE_LIMIT_CONFIGS =
      new SoftwareLimitSwitchConfigs()
        .withReverseSoftLimitThreshold(MIN_POSITION_ROTATIONS)
        .withForwardSoftLimitThreshold(MAX_POSITION_ROTATIONS)
        .withForwardSoftLimitEnable(false)
        .withReverseSoftLimitEnable(false);

    public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(MOTOR_TO_MECHANISM)
        .withRotorToSensorRatio(1.0);
  }

  public final class Hopper {
    public static final int CAN_ID = 50; 
    public static final String BUS = "rio";
    public static final double MOTOR_TO_MECHANISM = (16.0 / 12.0) * (24.0 / 16.0);
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration CONFIG = GenericRoller.COUNTERCLOCKWISE_CONFIG.clone();
  }

  public final class Indexer {
    public final static int CAN_ID = 51;
    public static final String BUS = "rio";

    // controls two rollers, so reduction is weird
    public static final double MOTOR_TO_MECHANISM = 36.0 / 16.0;
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration CONFIG =
      GenericRoller.COUNTERCLOCKWISE_CONFIG.clone()
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));
  }

  /* General Subsystems */
  public final class ShooterHood {
    public static final String BUS = "rio";
    public static final double SIM_MOI = 0.001;
    public static final int CAN_ID = 34;

    /* CANCoder */
    public static final int CANCODER_ID = 36;
    public static final double CANCODER_ABSOLUTE_OFFSET = 0.0;

    // spur = hood driving gear, mechanism = shooter hood gear
    public static final double MOTOR_TO_CANCODER = (40.0 / 12.0) * (24.0 / 17.0);
    public static final double CANCODER_TO_SPUR = 1.0;
    public static final double SPUR_TO_MECHANISM = (222.0 / 18.0);

    public static final double MOTOR_TO_SPUR = MOTOR_TO_CANCODER * CANCODER_TO_SPUR;
    public static final double CANCODER_TO_MECHANISM = CANCODER_TO_SPUR * SPUR_TO_MECHANISM;
    public static final double MOTOR_TO_MECHANISM = MOTOR_TO_CANCODER * CANCODER_TO_SPUR * SPUR_TO_MECHANISM;

    public static final double EPSILON_RADS = Units.degreesToRadians(0.5);
    public static final double STATE_TIMEOUT = 5.0;
    public static final boolean ENABLE_TIMEOUT = false;

    public static final double MIN_POSITION_ROTATIONS = 0.0 - Units.radiansToRotations(EPSILON_RADS);
    public static final double MAX_POSITION_ROTATIONS = 33.0 / 360.0 + Units.radiansToRotations(EPSILON_RADS);

    public static final double MIN_POSITION_RADS = Units.rotationsToRadians(MIN_POSITION_ROTATIONS);
    public static final double MAX_POSITION_RADS = Units.rotationsToRadians(MAX_POSITION_ROTATIONS);

    /* Configs */
    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(20)
            .withStatorCurrentLimit(20)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));


    public static final SoftwareLimitSwitchConfigs SOFTWARE_LIMIT_CONFIGS =
      new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitThreshold(MIN_POSITION_ROTATIONS)
            .withForwardSoftLimitThreshold(MAX_POSITION_ROTATIONS)
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false);

    public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs()
        .withSensorToMechanismRatio(CANCODER_TO_MECHANISM)
        .withRotorToSensorRatio(MOTOR_TO_CANCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withFeedbackRemoteSensorID(CANCODER_ID)
        .withFeedbackRotorOffset(-CANCODER_ABSOLUTE_OFFSET);

    public static final MagnetSensorConfigs CANCODER_CONFIGS =
      new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(1.0)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-1 * CANCODER_ABSOLUTE_OFFSET);
  }

  public final class Flywheel {
    public static final int CAN_ID = 30;

    public static final int FOLLOWER_CAN_ID = 31;
    public static final int OPPOSER_ONE_CAN_ID = 32;
    public static final int OPPOSER_TWO_CAN_ID = 33;

    public static final double FOLLOWER_SIM_MOI = 0.001;

    public static final double EPSILON_VELOCITY = 2.5;
    public static final double MOTOR_TO_MECHANISM = 15.0 / 26.0;
    public static final String BUS = "rio";
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast));

  public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(MOTOR_TO_MECHANISM)
        .withRotorToSensorRatio(1.0);
  }
}
