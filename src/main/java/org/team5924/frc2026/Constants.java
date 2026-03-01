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
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

  
  public static final boolean TUNING_MODE = false;
  public static final boolean ALLOW_ASSERTS = false;
  public static final double SLOW_MODE_MULTI = 0.5;

  /* General */
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

  public final class GenericRollerSystem {
    public static final double REDUCTION = 1.0;
    public static final double SIM_MOI = 0.001;
  }

  public final class Intake {
    public static final int CAN_ID = 20; // TODO: Set CAN ID
    public static final String BUS = "rio";
    public static final double REDUCTION = 32.0 / 16.0;
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
  }

  public final class IntakePivot {
    public static final int CAN_ID = 41; // TODO: update to real can id
    public static final String BUS = "rio";
    public static final double MOTOR_TO_MECHANISM = (54.0 / 20.0) * (32.0 / 12.0);
    public static final double SIM_MOI = 0.001;

    public static final double OFFSET_RADS = 0.0;

    /** how far the intake pivot physically rotates */
    public static final double MECHANISM_RANGE_PERCENT = 140.0 / 360.0; // TODO: have to update this value ??

    public static final double MIN_POSITION_MULTI = 0; // rotations
    public static final double MAX_POSITION_MULTI = 0.6; // rotations

    public static final double MIN_POSITION_RADS = Units.rotationsToRadians(MIN_POSITION_MULTI);
    public static final double MAX_POSITION_RADS = Units.rotationsToRadians(MAX_POSITION_MULTI);

    public static final double EPSILON_RADS = Units.degreesToRadians(2.0);

    public static final double STATE_TIMEOUT = 5.0;

    /* Configs */
    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60) // TODO: double check
            .withStatorCurrentLimit(60) // TODO: double check
            .withStatorCurrentLimitEnable(true))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

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

    public static final SoftwareLimitSwitchConfigs SOFTWARE_LIMIT_CONFIGS =
      new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(
          MIN_POSITION_MULTI * MOTOR_TO_MECHANISM) // motor? rotations
        .withReverseSoftLimitThreshold(
          MAX_POSITION_MULTI * MOTOR_TO_MECHANISM) // motor? rotations
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true);
  }

  public final class Hopper {
    public static final int CAN_ID = 21; 
    public static final String BUS = "rio";
    public static final double REDUCTION = (24.0 / 12.0) * (24.0 / 12.0); //TODO: If reduction is needed, update

    public static final int BEAM_BREAK_ID = 0;
    // public static final boolean BEAM_BREAK = false;
  
    // Hopper Motor Config
    public static final TalonFXConfiguration CONFIG = 
      new TalonFXConfiguration()
            .withCurrentLimits(
              new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withStatorCurrentLimit(60))
            .withMotorOutput(
              new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));
  }

  /* 
   * ### Shooters ###
   */

  public final class GeneralShooterHood {
    public static final double EPSILON_RADS = Units.degreesToRadians(2.0); // TODO: unused -> remove or use!
  }

  public final class GeneralTurret {
    public static final double EPSILON_RADS = Units.degreesToRadians(2.0); // TODO: unused -> remove or use!
    public static final double STATE_TIMEOUT = 5.0;
  }

  /* Left */
  public final class IndexerLeft { //TODO: update these later
    public final static int CAN_ID = 31;
    public final static int CAN_ID_INVERSE = 25;
    public final static int BEAM_BREAK_ID = 0;
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0;
    public static final double REDUCTION_INVERSE = 1.0;
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
  }

  public final class ShooterHoodLeft {
    public static final int CAN_ID = 22;
    public static final String BUS = "rio";
    public static final double REDUCTION = (40.0 / 12.0) * (24.0 / 15.0);
    public static final double SIM_MOI = 0.001;

    public static final double MOTOR_TO_CANCODER = (40.0 / 12.0) * (24.0 / 15.0);
    public static final double CANCODER_TO_SPUR = 1.0;
    public static final double SPUR_TO_MECHANISM = (23.0 / 18.0); // TODO: double check this

    /** how far the hood physically rotates */
    public static final double MECHANISM_RANGE_PERCENT = 30.0 / 360.0;

    public static final double MOTOR_TO_SPUR = MOTOR_TO_CANCODER * CANCODER_TO_SPUR;
    public static final double MOTOR_TO_MECHANISM = MOTOR_TO_SPUR * SPUR_TO_MECHANISM;
    public static final double CANCODER_TO_MECHANISM = MOTOR_TO_CANCODER * CANCODER_TO_SPUR * SPUR_TO_MECHANISM;

    public static final double MIN_POSITION_MULTI = 0.0; // TODO: make sure these are both set to the right values (rotations)
    public static final double MAX_POSITION_MULTI = 1.0;

    public static final double MIN_POSITION_RADS = Units.rotationsToRadians(MIN_POSITION_MULTI);
    public static final double MAX_POSITION_RADS = Units.rotationsToRadians(MAX_POSITION_MULTI);

    public static final double STATE_TIMEOUT = 5.0;

    /* CANCoder */
    public static final double CANCODER_ABSOLUTE_OFFSET = 0.0; // TODO: update!! (in rotations of cancoder)
    public static final int CANCODER_ID = 41; // TODO: update id

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
            .withInverted(InvertedValue.CounterClockwise_Positive) // TODO: test this direction
            .withNeutralMode(NeutralModeValue.Brake));

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

    public static final SoftwareLimitSwitchConfigs SOFTWARE_LIMIT_CONFIGS =
      new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(MOTOR_TO_MECHANISM * MIN_POSITION_MULTI) // TODO: get correct value for rotations
            .withReverseSoftLimitThreshold(MOTOR_TO_MECHANISM * MAX_POSITION_MULTI) // TODO: get correct value for rotations
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true);

    public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Constants.ShooterHoodLeft.CANCODER_ID)
        .withFeedbackRotorOffset(-Constants.ShooterHoodLeft.CANCODER_ABSOLUTE_OFFSET)
        .withSensorToMechanismRatio(Constants.ShooterHoodLeft.CANCODER_TO_SPUR)
        .withRotorToSensorRatio(Constants.ShooterHoodLeft.MOTOR_TO_CANCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    public static final MagnetSensorConfigs CANCODER_CONFIG =
      new MagnetSensorConfigs()
        .withMagnetOffset(-1 * CANCODER_ABSOLUTE_OFFSET) // TODO: update offset -> when the turret is facing forward (units: rotations)
        .withAbsoluteSensorDiscontinuityPoint(1.0) // TODO: update???
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
  }

  public final class ShooterRollerLeaderLeft {
    public static final int CAN_ID = 23; // TODO: Config later
    public static final String BUS = "rio";
    public static final double REDUCTION = 16.0 / 30.0;
    public static final double SIM_MOI = 0.001;
    public static final int BEAM_BREAK_PORT = 0;

    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
  }

  public final class ShooterRollerFollowerLeft {
    public static final int CAN_ID = 24; // TODO: Config later
    public static final String BUS = "rio";
    public static final double REDUCTION = 16.0 / 30.0;
    public static final double SIM_MOI = 0.001;
    public static final int BEAM_BREAK_ID = 0;
    // public static final boolean BEAM_BREAK = false;
  
    public static final TalonFXConfiguration CONFIG = 
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
  }

  public final class TurretLeft {
    public static final int CAN_ID = 40; // TODO: update to real can id
    public static final String BUS = "rio";

    public static final Translation3d ROBOT_TO_TURRET = new Translation3d(0.0, 0.0, 0.0); // TODO: update

    public static final double MOTOR_TO_CANCODER = (22.0 / 12.0); // TODO: change to 20 for new
    public static final double CANCODER_TO_MECHANISM = (135.0 / 22.0); // TODO: change to 20 for new  
    public static final double MOTOR_TO_MECHANISM = MOTOR_TO_CANCODER * CANCODER_TO_MECHANISM;
  
    public static final double SIM_MOI = 0.001;

    public static final double MIN_POSITION_MULTI = -0.4; // rotations
    public static final double MAX_POSITION_MULTI = 0.4; // rotations

    public static final double MIN_POSITION_RADS = Units.rotationsToRadians(MIN_POSITION_MULTI);
    public static final double MAX_POSITION_RADS = Units.rotationsToRadians(MAX_POSITION_MULTI);


    /* CANCoder */
    public static final int CANCODER_ID = 41; // TODO: update id
    public static final double CANCODER_ABSOLUTE_OFFSET = 0.0; // TODO: update!! (in rotations of cancoder)


    /* Configs */
    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60) // TODO: double check
            .withStatorCurrentLimit(60) // TODO: double check
            .withStatorCurrentLimitEnable(true))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

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

    public static final SoftwareLimitSwitchConfigs SOFTWARE_LIMIT_CONFIGS =
      new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(
          MIN_POSITION_MULTI * MOTOR_TO_MECHANISM) // motor? rotations
        .withReverseSoftLimitThreshold(
          MAX_POSITION_MULTI * MOTOR_TO_MECHANISM) // motor? rotations
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true);

    public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs()
        .withFeedbackRemoteSensorID(CANCODER_ID)
        .withFeedbackRotorOffset(CANCODER_ABSOLUTE_OFFSET)
        .withSensorToMechanismRatio(CANCODER_TO_MECHANISM)
        .withRotorToSensorRatio(MOTOR_TO_CANCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    public static final MagnetSensorConfigs CANCODER_CONFIG =
      new MagnetSensorConfigs()
        .withMagnetOffset(-CANCODER_ABSOLUTE_OFFSET) // TODO: update offset -> when the turret is facing forward (units: rotations)
        .withAbsoluteSensorDiscontinuityPoint(0.5)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
  }

  /* Right */
  public final class IndexerRight { //TODO: update these later
    public final static int CAN_ID = 31;
    public final static int CAN_ID_INVERSE = 25;
    public final static int BEAM_BREAK_ID = 0;
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0;
    public static final double REDUCTION_INVERSE = 1.0;
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
            .withNeutralMode(NeutralModeValue.Brake));
  }

  public final class ShooterHoodRight {
    public static final int CAN_ID = 22;
    public static final String BUS = "rio";
    public static final double REDUCTION = (40.0 / 12.0) * (24.0 / 17.0);
    public static final double SIM_MOI = 0.001;

    public static final double MOTOR_TO_CANCODER = (40.0 / 12.0) * (24.0 / 17.0);
    public static final double CANCODER_TO_SPUR = 1.0;
    public static final double SPUR_TO_MECHANISM = (23.0 / 18.0); // TODO: double check this
    public static final double CANCODER_TO_MECHANISM = MOTOR_TO_CANCODER * CANCODER_TO_SPUR * SPUR_TO_MECHANISM;

    /** how far the hood physically rotates */
    public static final double MECHANISM_RANGE_PERCENT = 30.0 / 360.0;

    public static final double MOTOR_TO_SPUR = MOTOR_TO_CANCODER * CANCODER_TO_SPUR;
    public static final double MOTOR_TO_MECHANISM = MOTOR_TO_SPUR * SPUR_TO_MECHANISM;

    public static final double MIN_POSITION_MULTI = 0.0; // TODO: make sure these are both set to the right values (rotations)
    public static final double MAX_POSITION_MULTI = 1.0;

    public static final double MIN_POSITION_RADS = Units.rotationsToRadians(MIN_POSITION_MULTI);
    public static final double MAX_POSITION_RADS = Units.rotationsToRadians(MAX_POSITION_MULTI);

    public static final double STATE_TIMEOUT = 5.0;

    /* CANCoder */
    public static final double CANCODER_ABSOLUTE_OFFSET = 0.0; // TODO: update!! (in rotations of cancoder)
    public static final int CANCODER_ID = 41; // TODO: update id

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
            .withInverted(InvertedValue.CounterClockwise_Positive) // TODO: test this direction
            .withNeutralMode(NeutralModeValue.Brake));

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

    public static final SoftwareLimitSwitchConfigs SOFTWARE_LIMIT_CONFIGS =
      new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(MOTOR_TO_MECHANISM * MIN_POSITION_MULTI) // TODO: get correct value for rotations
            .withReverseSoftLimitThreshold(MOTOR_TO_MECHANISM * MAX_POSITION_MULTI) // TODO: get correct value for rotations
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true);

    public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Constants.ShooterHoodRight.CANCODER_ID)
        .withFeedbackRotorOffset(-Constants.ShooterHoodRight.CANCODER_ABSOLUTE_OFFSET)
        .withSensorToMechanismRatio(Constants.ShooterHoodRight.CANCODER_TO_SPUR)
        .withRotorToSensorRatio(Constants.ShooterHoodRight.MOTOR_TO_CANCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    public static final MagnetSensorConfigs CANCODER_CONFIG =
      new MagnetSensorConfigs()
        .withMagnetOffset(-1 * CANCODER_ABSOLUTE_OFFSET) // TODO: update offset -> when the turret is facing forward (units: rotations)
        .withAbsoluteSensorDiscontinuityPoint(1.0) // TODO: update???
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
  }

  public final class ShooterRollerLeaderRight {
    public static final int CAN_ID = 23; // TODO: Config later
    public static final String BUS = "rio";
    public static final double REDUCTION = 16.0 / 30.0;
    public static final double SIM_MOI = 0.001;
    public static final int BEAM_BREAK_PORT = 0;

    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
  }

  public final class ShooterRollerFollowerRight {
    public static final int CAN_ID = 24; // TODO: Config later
    public static final String BUS = "rio";
    public static final double REDUCTION = 16.0 / 30.0;
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
            .withNeutralMode(NeutralModeValue.Brake));
  }

  public final class TurretRight {
    /* Turret */
    public static final int CAN_ID = 40; // TODO: update to real can id
    public static final String BUS = "rio";

    public static final Translation3d ROBOT_TO_TURRET = new Translation3d(0.0, 0.0, 0.0); // TODO: update

    public static final double MOTOR_TO_CANCODER = (22.0 / 12.0); // TODO: change to 20 for new
    public static final double CANCODER_TO_MECHANISM = (135.0 / 22.0); // TODO: change to 20 for new  
    public static final double MOTOR_TO_MECHANISM = MOTOR_TO_CANCODER * CANCODER_TO_MECHANISM;
  
    public static final double SIM_MOI = 0.001;

    public static final double MIN_POSITION_MULTI = -0.4; // rotations
    public static final double MAX_POSITION_MULTI = 0.4; // rotations

    public static final double MIN_POSITION_RADS = Units.rotationsToRadians(MIN_POSITION_MULTI);
    public static final double MAX_POSITION_RADS = Units.rotationsToRadians(MAX_POSITION_MULTI);

    public static final double EPSILON_RADS = Units.degreesToRadians(2.0); // TODO: unused -> remove or use!

    public static final double STATE_TIMEOUT = 5.0;


    /* CANCoder */
    public static final int CANCODER_ID = 41; // TODO: update id
    public static final double CANCODER_ABSOLUTE_OFFSET = 0.0; // TODO: update!! (in rotations of cancoder)


    /* Configs */
    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60) // TODO: double check
            .withStatorCurrentLimit(60) // TODO: double check
            .withStatorCurrentLimitEnable(true))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

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

    public static final SoftwareLimitSwitchConfigs SOFTWARE_LIMIT_CONFIGS =
      new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitThreshold(
          MIN_POSITION_MULTI * MOTOR_TO_MECHANISM) // motor? rotations
        .withReverseSoftLimitThreshold(
          MAX_POSITION_MULTI * MOTOR_TO_MECHANISM) // motor? rotations
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true);

    public static final FeedbackConfigs FEEDBACK_CONFIGS =
      new FeedbackConfigs()
        .withFeedbackRemoteSensorID(CANCODER_ID)
        .withFeedbackRotorOffset(CANCODER_ABSOLUTE_OFFSET)
        .withSensorToMechanismRatio(CANCODER_TO_MECHANISM)
        .withRotorToSensorRatio(MOTOR_TO_CANCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    public static final MagnetSensorConfigs CANCODER_CONFIG =
      new MagnetSensorConfigs()
        .withMagnetOffset(-CANCODER_ABSOLUTE_OFFSET) // TODO: update offset -> when the turret is facing forward (units: rotations)
        .withAbsoluteSensorDiscontinuityPoint(0.5)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
  }

}
