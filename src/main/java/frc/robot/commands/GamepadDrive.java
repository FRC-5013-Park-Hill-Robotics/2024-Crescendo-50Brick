// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimeLightConstants;
import frc.robot.constants.ThetaGains;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.trobot5013lib.AprilTagHelpers;

public class GamepadDrive extends Command {
	private CommandSwerveDrivetrain m_drivetrain;
	private CommandXboxController m_gamepad;
	private LimeLight m_limelight;
	private SlewRateLimiter xLimiter = new SlewRateLimiter(2.5);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(2.5);
	private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);
	private Alliance m_alliance;

	private PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);
	private double thetaOutput = 0;

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.maxAngularVelocityRadiansPerSecond * ControllerConstants.DEADBAND).withRotationalDeadband(DrivetrainConstants.maxAngularVelocityRadiansPerSecond * ControllerConstants.DEADBAND) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
	private final SwerveRequest.FieldCentricFacingAngle turnTo = new SwerveRequest.FieldCentricFacingAngle();
  	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	/**
	 * Constructor method for the GamepadDrive class
	 * - Creates a new GamepadDrive object.
	 */
	public GamepadDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController gamepad, LimeLight limelight) {
		super();
		addRequirements(drivetrain);
		m_gamepad = gamepad;
		m_drivetrain = drivetrain;
		m_limelight = limelight;
	}

	@Override
	public void execute() {

		double throttle = modifyAxis(m_gamepad.getRightTriggerAxis());

		if (m_gamepad.getLeftTriggerAxis() > 0.5){
			throttle = throttle/3;
		}

		double translationX = modifyAxis(-m_gamepad.getLeftY());
		double translationY = modifyAxis(-m_gamepad.getLeftX());
		if (!(translationX == 0.0 && translationY == 0.0)) {
			
			double angle = calculateTranslationDirection(translationX, translationY);
			translationX = Math.cos(angle) * throttle;
			translationY = Math.sin(angle) * throttle;
		}

		/* Turn while driving
		if(m_gamepad.x().getAsBoolean()){
			//m_drivetrain.setControl(turnTo.withTargetDirection());
			if(Alliance.Red == DriverStation.getAlliance().get()){
				m_limelight.setPipeline(LimeLightConstants.APRIL_TAG_RED_SPEAKER);
			} else{
				m_limelight.setPipeline(LimeLightConstants.APRIL_TAG_BLUE_SPEAKER);
			}
			double horizontal_angle = -m_limelight.getHorizontalAngleOfErrorDegrees() ;
			double setpoint = Math.toRadians(horizontal_angle)+ m_drivetrain.getPose().getRotation().getRadians();
      		thetaController.setSetpoint(setpoint);

			if (!thetaController.atSetpoint() ){
				thetaOutput = thetaController.calculate(m_drivetrain.getPose().getRotation().getRadians(), setpoint);
			}
		} else {
			m_limelight.setPipeline(LimeLightConstants.APRIL_TAG_TARGETING);
			thetaOutput = -CommandSwerveDrivetrain.percentOutputToRadiansPerSecond(rotationLimiter.calculate(m_gamepad.getRightX()/2));
		}*/
		
		thetaOutput = -CommandSwerveDrivetrain.percentOutputToRadiansPerSecond(rotationLimiter.calculate(m_gamepad.getRightX()/2));

		//Applied %50 reduction to rotation
		m_drivetrain.setControl(drive
			.withVelocityX(-CommandSwerveDrivetrain.percentOutputToMetersPerSecond(xLimiter.calculate(translationX)))
			.withVelocityY(CommandSwerveDrivetrain.percentOutputToMetersPerSecond(yLimiter.calculate(translationY))) 
			.withRotationalRate(thetaOutput));
		

		SmartDashboard.putNumber("Throttle", throttle);
		SmartDashboard.putNumber("Drive Rotation",-CommandSwerveDrivetrain.percentOutputToRadiansPerSecond(m_gamepad.getRightX()) );
		SmartDashboard.putNumber("VX", CommandSwerveDrivetrain.percentOutputToMetersPerSecond(xLimiter.calculate(translationX)));
		SmartDashboard.putNumber("VY", CommandSwerveDrivetrain.percentOutputToMetersPerSecond(yLimiter.calculate(translationY)));
		
	 }

	@Override
	public void end(boolean interrupted) {
		m_drivetrain.applyRequest(() -> brake);
	}

	private static double modifyAxis(double value) {
		return modifyAxis(value, 1);
	}
	private static double modifyAxis(double value, int exponent) {
		// Deadband
		value = MathUtil.applyDeadband(value, ControllerConstants.DEADBAND);

		 value = Math.copySign(Math.pow(value, exponent), value);

		return value;
	}
	
	private double calculateTranslationDirection(double x, double y) {
		// Calculate the angle.
		// Swapping x/y
		return Math.atan2(x, y) + Math.PI / 2;
	}

	
}
