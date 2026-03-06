// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
