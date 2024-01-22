// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class FieldCentricSecondOrder  extends SwerveRequest.FieldCentric implements SwerveRequest{
    private double skewAdjustment = 0;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        double toApplyX = VelocityX;
        double toApplyY = VelocityY;
        double toApplyOmega = RotationalRate;
        if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }
        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
        }

        ChassisSpeeds speeds = ChassisSpeeds.discretize(secondOrderKinematics(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                    parameters.currentPose.getRotation())), parameters.updatePeriod);

        var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
        }

        return StatusCode.OK;
    }

    public FieldCentricSecondOrder withSkewAdjustment(double skewAdjustment){
        this.skewAdjustment = skewAdjustment;
        return this;
    }

    public ChassisSpeeds secondOrderKinematics(ChassisSpeeds chassisSpeeds){
        if (skewAdjustment !=  0 ){
            Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
            Translation2d rotAdj= translation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(chassisSpeeds.omegaRadiansPerSecond*0.045);

            translation = translation.plus(rotAdj);

            return new ChassisSpeeds(translation.getX(),translation.getY(),chassisSpeeds.omegaRadiansPerSecond);
        } else {
            return chassisSpeeds;
        }
  }

}
