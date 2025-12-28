/*
 * EqualsUtil.java
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

import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-9);
    }

    /** Extension methods for wpi geometry objects */
    public static class GeomExtensions {
        public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
            return EqualsUtil.epsilonEquals(twist.dx, other.dx)
                    && EqualsUtil.epsilonEquals(twist.dy, other.dy)
                    && EqualsUtil.epsilonEquals(twist.dtheta, other.dtheta);
        }
    }
}
