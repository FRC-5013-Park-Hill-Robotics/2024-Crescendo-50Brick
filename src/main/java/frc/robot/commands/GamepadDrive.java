// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GamepadDrive extends Command {
	private CommandSwerveDrivetrain m_drivetrain;
	private CommandXboxController m_gamepad;
	private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
	private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

	private double MaxSpeed = 6; // 6 meters per second desired top speed
	private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity  

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	/**
	 * Constructor method for the GamepadDrive class
	 * - Creates a new GamepadDrive object.
	 */
	public GamepadDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController gamepad) {
		super();
		addRequirements(drivetrain);
		m_gamepad = gamepad;
		m_drivetrain = drivetrain;
	}

	@Override
	public void execute() {

		double throttle = modifyAxis(m_gamepad.getRightTriggerAxis());

		if (m_gamepad.leftBumper().getAsBoolean()){
			throttle = throttle/3;
		}


		double translationX = modifyAxis(-m_gamepad.getLeftY());
		double translationY = modifyAxis(-m_gamepad.getLeftX());
		if (!(translationX == 0.0 && translationY == 0.0)) {
			
			double angle = calculateTranslationDirection(translationX, translationY);
			translationX = Math.cos(angle) * throttle;
			translationY = Math.sin(angle) * throttle;
		}

		double h = xLimiter.calculate(translationX);
		double k = yLimiter.calculate(translationY);
		m_drivetrain.applyRequest(()->drive
			.withVelocityX(h)
			.withVelocityY(k) 
			.withRotationalRate(-m_gamepad.getRightX() * MaxAngularRate));
		
		/*(ChassisSpeeds.fromFieldRelativeSpeeds(
				-m_drivetrain.percentOutputToMetersPerSecond(xLimiter.calculate(translationX)),
				m_drivetrain.percentOutputToMetersPerSecond(yLimiter.calculate(translationY)), getRotationRadiansPerSecond(),
				m_drivetrain.getYawR2d()));*/

		SmartDashboard.putNumber("Drive Rotation", -m_gamepad.getRightX() * MaxAngularRate);
		
		/*
		 * m_drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
		 * getXTranslationMetersPerSecond(),
		 * getYTranslationMetersPerSecond(), getRotationRadiansPerSecond(),
		 * m_drivetrain.getGyroscopeRotation()));
		 */ }

	@Override
	public void end(boolean interrupted) {
		m_drivetrain.applyRequest(() -> brake);
	}

	/*private double getXTranslationMetersPerSecond() {
		// on the controller y is up, on the field x is away from the driver
		return -Drivetrain
				.percentOutputToMetersPerSecond(xLimiter.calculate(modifyAxis(m_gamepad.getLeftY())));
	}

	private double getYTranslationMetersPerSecond() {
		// on the controller y is up, on the field x is away from the driver
		return -Drivetrain
				.percentOutputToMetersPerSecond(yLimiter.calculate(modifyAxis(m_gamepad.getLeftX())));
	}

	private double getRotationRadiansPerSecond() {
		return -Drivetrain
				.percentOutputToRadiansPerSecond(rotationLimiter.calculate(modifyAxis(m_gamepad.getRightX(),2))) / 3;

	}*/

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
