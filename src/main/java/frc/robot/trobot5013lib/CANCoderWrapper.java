// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** Add your docs here. */
public class CANCoderWrapper {
    private CANcoder encoder; 
    private boolean inverted = false;

    public CANCoderWrapper () {
        super();
    }


    public CANCoderWrapper (int canID) {
       this(canID, false,0);
    }

    public CANCoderWrapper(int canID, boolean inverted, double offsetRotations) {
        super();
        this.encoder = new CANcoder(canID);
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = offsetRotations;
        canCoderConfig.MagnetSensor.SensorDirection = inverted?SensorDirectionValue.CounterClockwise_Positive:SensorDirectionValue.Clockwise_Positive;
        encoder.getConfigurator().apply(canCoderConfig);
    }

    public double getAbsPositionRadians() {
        double absPosition = encoder.getAbsolutePosition().getValueAsDouble();
        if (inverted) {
            absPosition = 1 - absPosition;
        }
        absPosition = absPosition % 1;
        return rotationsToRadians(absPosition) % (Math.PI * 2);
    }

    private double rotationsToRadians(double rotations) {
        return 2 * Math.PI * rotations;
    }
    public double getVelocityRadians() {
        return rotationsToRadians(2 * Math.PI * encoder.getVelocity().getValueAsDouble());
    }

    public CANcoder getCanandcoder() {
        return this.encoder;
    }
    public void setCanandcoder(CANcoder encoder) {
        this.encoder = encoder;
    }
}
