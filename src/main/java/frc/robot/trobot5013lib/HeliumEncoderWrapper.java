// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.trobot5013lib;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.reduxrobotics.sensors.canandcoder.Canandcoder.Settings;*/

/** Add your docs here. *//*
public class HeliumEncoderWrapper {
    private Canandcoder encoder; 
    private boolean inverted = false;

    public HeliumEncoderWrapper () {
        super();
    }


    public HeliumEncoderWrapper (int canID) {
       this(canID, false);
    }

    public HeliumEncoderWrapper(int canID, boolean inverted) {
        super();
        this.encoder = new Canandcoder(canID);
        this.inverted = inverted;
    }

    public double getAbsPositionRadians() {
        double absPosition = encoder.getAbsPosition();
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
        return rotationsToRadians(2 * Math.PI * encoder.getVelocity());
    }

    public Canandcoder getCanandcoder() {
        return this.encoder;
    }
    public void setCanandcoder(Canandcoder encoder) {
        this.encoder = encoder;
    }
}*/
