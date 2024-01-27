// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public final class DrivetrainConstants {
    public static final double maxSpeedMetersPerSecond = TunerConstants.kSpeedAt12VoltsMps;
    public static final double trackWidthMeters = Units.inchesToMeters(18.78);
    public static final double rotationDiameter = trackWidthMeters * Math.PI * Math.sqrt(2);
    public static final double rotationsPerSecond = maxSpeedMetersPerSecond / rotationDiameter;
    public static final double maxAngularVelocityRadiansPerSecond = 2 * Math.PI * rotationsPerSecond;
}
