// package org.team5924.frc2026.subsystems.leds;

// import java.util.List;
// import java.util.Optional;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.Pair;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Leds extends SubsystemBase {

//   // Lets all classes access LEDs by calling Leds.getInstance() (singleton)
//   private static Leds instance;
//   public static Leds getInstance() {
//     if (instance == null) {
//       instance = new Leds();
//     }
//     return instance;
//   }

//   //Robot State Tracking
  

//   // Constants to update
//   private static final boolean prideLeds = false;
//   private static final int minLoopCycleCount = 10;
//   private static final int fullLength = 42;
//   private static final int length = 32;
//   private static final int sideSectionLength = 21;
//   private static final int virtualSectionLength = 8;
//   private static final int virtualLength = length + virtualSectionLength;
//   private static final Section virtualSection =
//       new Section(sideSectionLength, sideSectionLength + virtualSectionLength);
//   private static final Section fullSection = new Section(0, virtualLength);
//   private static final Section firstPrioritySection = new Section(virtualLength / 5, virtualLength);
//   private static final Section secondPrioritySection = new Section(0, virtualLength / 5);
//   private static final Section sideSection = new Section(0, sideSectionLength);
//   private static final Section backSection = new Section(sideSectionLength, virtualLength);
//   private static final double strobeDuration = 0.1;
//   private static final double strobeSlowDuration = 0.2;
//   private static final double breathFastDuration = 0.5;
//   private static final double breathSlowDuration = 1.0;
//   private static final double rainbowCycleLength = 25.0;
//   private static final double rainbowDuration = 0.25;
//   private static final double rainbowStrobeDuration = 0.2;
//   private static final double waveExponent = 0.4;
//   private static final double waveFastCycleLength = 25.0;
//   private static final double waveFastDuration = 0.25;
//   private static final double waveDisabledCycleLength = 15.0;
//   private static final double waveDisabledDuration = 2.0;
//   private static final double autoFadeMaxTime = 5.0; // Return to normal
//   private static final Color l1PriorityColor = Color.kOrangeRed;
//   private static final Color l2PriorityColor = Color.kCyan;
//   private static final Color l3PriorityColor = Color.kBlue;
//   private static final Color l4PriorityColor = Color.kPurple;
//   private static final double dimMultiplier = 0.1;


//   // References to hardware led implementation
//   private final AddressableLED leds;
//   private final AddressableLEDBuffer buffer;
//   private final Notifier loadingNotifier;


//   // copy paste constants


  
//   private Leds() {
//     // declare the hardware implementation
//     leds = new AddressableLED(8);
    
//   }



//   // everything from here down was copied and pasted, so someone should go over it
//   private Color solid(Section section, Color color) {
//     if (color != null) {
//       for (int i = section.start(); i < section.end(); i++) {
//         setLED(i, color);
//       }
//     }
//     return color;
//   }

//   private Color strobe(Section section, Color c1, Color c2, double duration) {
//     boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
//     if ((c1On && c1 == null) || (!c1On && c2 == null)) return null;
//     return solid(section, c1On ? c1 : c2);
//   }

//   private Color breath(Section section, Color c1, Color c2, double duration, double timestamp) {
//     Color color = breathCalculate(section, c1, c2, duration, timestamp);
//     solid(section, color);
//     return color;
//   }

//   private Color breathCalculate(
//       Section section, Color c1, Color c2, double duration, double timestamp) {
//     double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
//     double ratio = (Math.sin(x) + 1.0) / 2.0;
//     double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
//     double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
//     double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
//     var color = new Color(red, green, blue);
//     return color;
//   }

//   private void rainbow(Section section, double cycleLength, double duration) {
//     double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
//     double xDiffPerLed = 180.0 / cycleLength;
//     for (int i = section.end() - 1; i >= section.start(); i--) {
//       x += xDiffPerLed;
//       x %= 180.0;
//       setHSV(i, (int) x, 255, 255);
//     }
//   }

//   private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
//     double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
//     double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
//     x += xDiffPerLed * (virtualLength - section.end());
//     for (int i = section.end() - 1; i >= section.start(); i--) {
//       x += xDiffPerLed;
//       double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
//       if (Double.isNaN(ratio)) {
//         ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
//       }
//       if (Double.isNaN(ratio)) {
//         ratio = 0.5;
//       }
//       double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
//       double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
//       double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
//       setLED(i, new Color(red, green, blue));
//     }
//   }

//   private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
//     int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
//     for (int i = section.end() - 1; i >= section.start(); i--) {
//       int colorIndex =
//           (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
//       colorIndex = colors.size() - 1 - colorIndex;
//       var dimmedColor =
//           shouldDimSupplier.getAsBoolean()
//               ? Color.lerpRGB(Color.kBlack, colors.get(colorIndex), dimMultiplier)
//               : colors.get(colorIndex);
//       setLED(i, dimmedColor);
//     }
//   }

//   private Color renderPriority(Optional<ReefLevel> level, Boolean blocked, Section section) {
//     Color primaryColor =
//         level.isEmpty()
//             ? Color.kBlack
//             : switch (level.get()) {
//               case L1 -> l1PriorityColor;
//               case L2 -> l2PriorityColor;
//               case L3 -> l3PriorityColor;
//               case L4 -> l4PriorityColor;
//             };
//     if (!blocked) {
//       return primaryColor;
//     } else {
//       return breathCalculate(
//           section,
//           primaryColor,
//           Color.lerpRGB(primaryColor, Color.kBlack, 0.9),
//           breathFastDuration,
//           Timer.getTimestamp());
//     }
//   }

//   private void setHSV(int index, int h, int s, int v) {
//     setLED(index, Color.fromHSV(h, s, v));
//   }

//   private void setLED(int index, Color color) {
//     var indices = getIndices(index);
//     var dimmedColor =
//         shouldDimSupplier.getAsBoolean()
//             ? Color.lerpRGB(Color.kBlack, color, dimMultiplier)
//             : color;
//     if (indices.getFirst() >= 0) buffer.setLED(indices.getFirst(), dimmedColor);
//     if (indices.getSecond() >= 0) buffer.setLED(indices.getSecond(), dimmedColor);
//   }

//   private static Pair<Integer, Integer> getIndices(int index) {
//     if (index >= virtualSection.start() && index < virtualSection.end()) {
//       return Pair.of(-1, -1);
//     } else if (index >= virtualSection.end()) {
//       index -= virtualSectionLength;
//     }
//     int a = MathUtil.clamp(index, 0, fullLength - 1);
//     int b = MathUtil.clamp(fullLength + sideSectionLength - index - 1, 0, 2 * length - 1);
//     if (b >= fullLength) b = -1;
//     return Pair.of(a, b);
//   }


//   private record Section(int start, int end) {}
// }
