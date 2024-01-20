// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.trobot5013lib.LinearInterpolator;

/** Add your docs here. */
public final class InterpolationConstants {
    //Was originally in feet, converted to meters, hence why the second number looks weird
    public static final double[][] GAME_PIECE_DISTANCE = {
        {-16.37, 0.3048},
        {-14.8, 0.4572},
        {-10.2, 0.6096},
        {-7.85, 0.762},
        {-6.13, 0.9144},
        {-4.27, 1.0668},
        {-3.53, 1.2192},
        {-2.95, 1.3716},
        {-2.08, 1.524}
    };
    public static final LinearInterpolator GAME_PIECE_INTERPOLATOR = new LinearInterpolator(GAME_PIECE_DISTANCE);
}
