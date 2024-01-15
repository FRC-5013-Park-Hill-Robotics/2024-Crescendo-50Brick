// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimeLightConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.constants.ThetaGains;
import frc.robot.subsystems.LimeLight;

public class AllignOnGamePiece extends CommandBase {
  private LimeLight m_LimeLight;
  private CommandSwerveDrivetrain m_Drivetrain;
  private PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);
  public AllignOnGamePiece(CommandSwerveDrivetrain drivetrain, LimeLight limelight) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_LimeLight = limelight;
  }
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(DrivetrainConstants.maxSpeed * 0.1).withRotationalDeadband(DrivetrainConstants.maxAngularVelocity * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(LimeLightConstants.ALLIGNMENT_TOLLERANCE_RADIANS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaOutput = 0;
    double xOutput = 0;
    double yOutput = 0;
		if (m_LimeLight.hasTarget()){
			double vertical_angle = m_LimeLight.getVerticalAngleOfErrorDegrees();
			double horizontal_amgle = -m_LimeLight.getHorizontalAngleOfErrorDegrees() ;
			double setpoint = Math.toRadians(horizontal_amgle)+ m_Drivetrain.getPose().getRotation().getRadians();
      thetaController.setSetpoint(setpoint);

			if (!thetaController.atSetpoint() ){
				thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
			} else {

      }
		} else {
			System.out.println("NO TARGET");
		}
    m_Drivetrain.setControl(drive.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(thetaOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
