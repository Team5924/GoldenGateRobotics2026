/*
 * LEDs.java
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

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.team5924.frc2026.Constants;
import org.team5924.frc2026.subsystems.LEDs.LEDsIO.LEDsIOOutputs;

public class LEDs extends SubsystemBase {
  private static LEDs global;

  public static LEDs getGlobal() {
    return global;
  }

  // Robot state tracking
  public boolean lowBatteryAlert = false;
  public boolean superstructureCoast = false;
  private boolean estopped = false;
  private Optional<Alliance> alliance = Optional.empty();
  private Color disabledColor = Color.kGold;
  private Color secondaryDisabledColor = Color.kDarkBlue;

  private final LEDsIO io;
  private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();
  private final LEDsIOOutputs outputs = new LEDsIOOutputs();

  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDs.length);
  private Field bufferField = null;

  // Constants
  private static final Section fullSection = new Section(0, Constants.LEDs.length);

  // private static final String LoggedTracer = null;

  public LEDs(LEDsIO io) {
    this.io = io;
    global = this;

    try {
      bufferField = AddressableLEDBuffer.class.getDeclaredField("m_buffer");
      bufferField.setAccessible(true);
    } catch (NoSuchFieldException | SecurityException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);

    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      disabledColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(disabledColor);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : secondaryDisabledColor;
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Select LED mode
    solid(fullSection, Color.kBlack); // Default to off

    // Update pattern
    if (estopped) {
      solid(fullSection, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (superstructureCoast) {
        // Superstructure coast
        solid(fullSection, Color.kWhite);
      } else if (lowBatteryAlert) {
        // Low battery
        strobe(fullSection, Color.kOrangeRed, Color.kBlack, Constants.LEDs.strobeDuration);
      } else if (Constants.LEDs.prideLeds) {
        // Pride stripes
        stripes(
            fullSection,
            List.of(
                Color.kBlack,
                Color.kRed,
                Color.kOrangeRed,
                Color.kYellow,
                Color.kGreen,
                Color.kBlue,
                Color.kPurple,
                Color.kBlack,
                new Color(0.15, 0.3, 1.0),
                Color.kDeepPink,
                Color.kWhite,
                Color.kDeepPink,
                new Color(0.15, 0.3, 1.0)),
            3,
            5.0);
      } else {
        // Default pattern
        wave(
            fullSection,
            disabledColor,
            secondaryDisabledColor,
            Constants.LEDs.waveDisabledCycleLength,
            Constants.LEDs.waveDisabledDuration);
      }
    } else if (DriverStation.isAutonomous()) {
      wave(
          fullSection,
          Color.kGold,
          Color.kDarkBlue,
          Constants.LEDs.waveFastCycleLength,
          Constants.LEDs.waveFastDuration);
    } else {
      // Not implemented
    }

    // Override with loading animation
    if (Timer.getTimestamp() < 30.0 && !estopped) {
      breath(
          fullSection,
          Color.kBlack,
          Color.kWhite,
          Constants.LEDs.startupBreathDuration,
          Timer.getTimestamp());
    }

    // Send to buffer
    if (bufferField == null) return;
    try {
      outputs.buffer = (byte[]) bufferField.get(buffer);

    } catch (IllegalArgumentException | IllegalAccessException e) {
      e.printStackTrace();
    }

    // Apply outputs to hardware
    io.applyOutputs(outputs);

    // Record cycle time
    // LoggedTracer.record("Leds/Periodic");
  }

  private Color solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        setLED(i, color);
      }
    }
    return color;
  }

  private Color strobe(Section section, Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    if ((c1On && c1 == null) || (!c1On && c2 == null)) return null;
    return solid(section, c1On ? c1 : c2);
  }

  private Color breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    Color color = breathCalculate(section, c1, c2, duration, timestamp);
    solid(section, color);
    return color;
  }

  private Color breathCalculate(
      Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    return color;
  }

  @SuppressWarnings("unused")
  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      x %= 180.0;
      setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    x += xDiffPerLed * (Constants.LEDs.length - section.end());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), Constants.LEDs.waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), Constants.LEDs.waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      setLED(i, colors.get(colorIndex));
    }
  }

  private void setHSV(int index, int h, int s, int v) {
    setLED(index, Color.fromHSV(h, s, v));
  }

  private void setLED(int index, Color color) {
    buffer.setLED(index, color);
  }

  private record Section(int start, int end) {}
}
