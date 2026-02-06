package org.team5924.frc2026.subsystems.turret;

import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.units.measure.Angle;

public class TurretIOTalonFX implents turretIO {

  private final TalonFX turretTalon;
  private final StatusSignal<Angle> examplePosition;
  private final StatusSignal<AngularVelocity> exampleVelocity;
  private final StatusSignal<Voltage> exampleAppliedVoltage;
  private final StatusSignal<Current> exampleSupplyCurrent;
  private final StatusSignal<Current> exampleTorqueCurrent;
  private final StatusSignal<Temperature> exampleTempCelsius;

  private final DigitalInput beamBreak = new DigitalInput(0);

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0).withUpdateFreqHz(0.0).withEnableFOC(true);

  public ExampleSystemIOTalonFX() {
    exampleTalon = new TalonFX(Constants.Example.CAN_ID, new CANBus(Constants.Example.BUS));
    exampleTalon.getConfigurator().apply(Constants.Example.CONFIG);

    // Get select status signals and set update frequency
    examplePosition = exampleTalon.getPosition();
    exampleVelocity = exampleTalon.getVelocity();
    exampleAppliedVoltage = exampleTalon.getMotorVoltage();
    exampleSupplyCurrent = exampleTalon.getSupplyCurrent();
    exampleTorqueCurrent = exampleTalon.getTorqueCurrent();
    exampleTempCelsius = exampleTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        examplePosition,
        exampleVelocity,
        exampleAppliedVoltage,
        exampleSupplyCurrent,
        exampleTorqueCurrent,
        exampleTempCelsius);

    exampleTalon.setPosition(0);
  }

  @Override
  public void updateInputs(ExampleSystemIOInputs inputs) {
    inputs.exampleMotorConnected =
        BaseStatusSignal.refreshAll(
                examplePosition,
                exampleVelocity,
                exampleAppliedVoltage,
                exampleSupplyCurrent,
                exampleTorqueCurrent,
                exampleTempCelsius)
            .isOK();
    inputs.examplePositionRads =
        Units.rotationsToRadians(examplePosition.getValueAsDouble()) / Constants.Example.REDUCTION;
    inputs.exampleVelocityRadsPerSec =
        Units.rotationsToRadians(exampleVelocity.getValueAsDouble()) / Constants.Example.REDUCTION;
    inputs.exampleAppliedVoltage = exampleAppliedVoltage.getValueAsDouble();
    inputs.exampleSupplyCurrentAmps = exampleSupplyCurrent.getValueAsDouble();
    inputs.exampleTorqueCurrentAmps = exampleTorqueCurrent.getValueAsDouble();
    inputs.exampleTempCelsius = exampleTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    exampleTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    exampleTalon.stopMotor();
  }
  
}
