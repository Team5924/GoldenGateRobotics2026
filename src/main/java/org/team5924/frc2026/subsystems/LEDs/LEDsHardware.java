package org.team5924.frc2026.subsystems.LEDs;

import org.team5924.frc2026.Constants;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PWMJNI;

public class LEDsHardware implements LEDsIO {
  private final int handle;

  public LEDsHardware() {
    int pwmHandle = PWMJNI.initializePWMPort(HAL.getPort((byte) Constants.LEDs.port));
    handle = AddressableLEDJNI.initialize(pwmHandle);
    AddressableLEDJNI.setLength(handle, Constants.LEDs.length);
    AddressableLEDJNI.start(handle);
  }

  @Override
  public void applyOutputs(LEDsIOOutputs outputs) {
    AddressableLEDJNI.setData(handle, outputs.buffer);
  }
}