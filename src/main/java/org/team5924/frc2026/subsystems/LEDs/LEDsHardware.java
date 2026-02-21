/*
 * LEDsHardware.java
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

package org.team5924.frc2026.subsystems.LEDs;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PWMJNI;
import org.team5924.frc2026.Constants;

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
