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

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
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

  //   /* Field */
  //   public static final double FIELD_BORDER_MARGIN = 0.5;
  //   public static final AprilTagFieldLayout field =
  //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  //   public static final double FIELD_WIDTH = field.getFieldWidth();
  //   public static final double FIELD_LENGTH = field.getFieldLength();
  //   public static final double CORAL_STATION_RADIANS_NORMAL = 0.959931;

  /* Hopper Agitator */
  public final class Hopper {
    public static final int CAN_ID = 0; //TODO: Update value to real ID 
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0; //TODO: If reduction is needed, update

    // public static final int BEAM_BREAK_ID = 0;
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
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
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

  public final class GenericRollerSystem {
    public static final double REDUCTION = 1.0;
    public static final double SIM_MOI = 0.001;
  }

  public final class ExampleRoller {
    public static final int CAN_ID = 0;
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0;
    public static final double SIM_MOI = 0.001;

    public static final int BEAM_BREAK_ID = 1;

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

  public final class ShooterHood {
    public static final int CAN_ID = 0; // TODO: Add CANID Ports
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0;
    public static final double SIM_MOI = 0.001;
    
    public static final TalonFXConfiguration CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
        .withMotorOutput(
          new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) // TODO: test this direction
            .withNeutralMode(NeutralModeValue.Brake))
        .withSlot0(
          new Slot0Configs() // TODO: TUNE THESE VALUES
            .withKP(1)
            .withKI(0)
            .withKD(0)
            .withKS(0) // TODO: ask CAD for these values later
            .withKV(0));
  }

  public final class ShooterRoller {
    public static final int CAN_ID = 0; // TODO: Add CANID Ports + Config later
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0;
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

  public final class Intake {
    public static final int CAN_ID = 0; // TODO: Set CAN ID
    public static final String BUS = "rio";
    public static final double REDUCTION = 1.0;
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

  public final class Indexer { //TODO: update these later
    public final static int CAN_ID = 0;
    public final static int CAN_ID_INVERSE = 0;
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
  public final class Turret {
    public static final int TURRET_CAN_ID = 0;
    public static final String BUS = "rio";
    public static final double TURRET_REDUCTION = 1.0;
    public static final double SIM_MOI = 0.001;

    public static final TalonFXConfiguration TURRET_CONFIG =
      new TalonFXConfiguration()
        .withCurrentLimits(
          new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimit(60))
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
            .withMaxLegalAngle(newValue 359);
  }
}


