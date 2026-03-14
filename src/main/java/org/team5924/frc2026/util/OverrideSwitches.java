/*
 * OverrideSwitches.java
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

package org.team5924.frc2026.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Wrapper for physical override switches on operator console. */
public class OverrideSwitches {
  private final GenericHID joystick;

  public OverrideSwitches(int port) {
    joystick = new GenericHID(port);
  }

  /** Returns whether the controller is connected. */
  public boolean isConnected() {
    return joystick.isConnected();
  }

  /** Returns a Trigger for switch 1 (button 1). */
  public Trigger switch1() {
    return new Trigger(() -> joystick.getRawButton(1));
  }

  /** Returns a Trigger for switch 2 (button 2). */
  public Trigger switch2() {
    return new Trigger(() -> joystick.getRawButton(2));
  }

  /** Returns a Trigger for switch 3 (button 3). */
  public Trigger switch3() {
    return new Trigger(() -> joystick.getRawButton(3));
  }

  /** Returns a Trigger for switch 4 (button 4). */
  public Trigger switch4() {
    return new Trigger(() -> joystick.getRawButton(4));
  }

  /** Returns the raw button state for a specific switch (1-indexed). */
  public boolean getSwitchState(int switchNumber) {
    return joystick.getRawButton(switchNumber);
  }

  /** Returns a Trigger for any button index. Useful for adding more switches dynamically. */
  public Trigger getSwitch(int buttonIndex) {
    return new Trigger(() -> joystick.getRawButton(buttonIndex));
  }
}
