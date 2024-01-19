// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.trobot5013lib.LinearInterpolator;

/** Add your docs here. */
public final class InterpolationConstants {
    public static final double[][] GAME_PIECE_DISTANCE = {
        {-13,140},
        {-12,140},
        {-2,140},
        {0,140},
        {4.5, 140},
        {5,140},
        {7,140},
        {11,140}
    };
    public static final LinearInterpolator GAME_PIECE_INTERPOLATOR = new LinearInterpolator(GAME_PIECE_DISTANCE);
}
