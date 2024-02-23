// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.InterpolationConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.ThetaGains;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/** Add your docs here. */
public class DriveToLLTarget extends Command {

  private Limelight m_Limelight;
  private CommandSwerveDrivetrain m_Drivetrain;
  private Pose2d m_Game_Piece_Pose;

  private double m_horizontal_angle;
  
  private double m_speed_modifier = 1;

  private PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2.8, 1.5);
  private ProfiledPIDController xController = new ProfiledPIDController(4,0.0,0.3, constraints);
  private ProfiledPIDController yController = new ProfiledPIDController(4,0.0,0.3, constraints);


  public DriveToLLTarget(CommandSwerveDrivetrain drivetrain, Limelight Limelight) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_Limelight = Limelight;
  }

  public DriveToLLTarget(CommandSwerveDrivetrain drivetrain, Limelight Limelight, double speed_modifier) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_Limelight = Limelight;
    m_speed_modifier = speed_modifier;
  }
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(LimelightConstants.ALLIGNMENT_TOLLERANCE_RADIANS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double thetaOutput = 0;
    double xOutput = 0;
    double yOutput = 0;
    
  	if (m_Limelight.hasTarget()){
      Translation2d t = new Translation2d(InterpolationConstants.GAME_PIECE_INTERPOLATOR.getInterpolatedValue(m_Limelight.getTy().getDouble(0.0)), 0);
      Rotation2d r = new Rotation2d(m_Drivetrain.getRotation3d().getAngle()+m_Limelight.getTxAngleRadians());
      Transform2d i = new Transform2d(t, r);
      m_Game_Piece_Pose = m_Drivetrain.getPose().transformBy(i);

      m_horizontal_angle = m_Limelight.getHorizontalAngleOfErrorDegrees();
    } else {
			System.out.println("NO TARGET");
		}

    //xController.setSetpoint(setpoint_x); when it was a normal pid controller
		xOutput = xController.calculate(m_Game_Piece_Pose.getX(), m_Drivetrain.getPose().getX());
    yOutput = yController.calculate(m_Game_Piece_Pose.getY(), m_Drivetrain.getPose().getY());

		double setpoint = m_Drivetrain.getPose().getRotation().getRadians() - Math.toRadians(m_horizontal_angle);
    thetaController.setSetpoint(setpoint);
    if (!thetaController.atSetpoint() ){
			thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
		}

    SmartDashboard.putNumber("xOutput", xOutput);
    SmartDashboard.putNumber("yOutput", yOutput);
    SmartDashboard.putNumber("Robot Angle", Math.toDegrees(m_Drivetrain.getPose().getRotation().getRadians()));
    SmartDashboard.putNumber("Calculated Angle", Math.toDegrees(setpoint));

    m_Drivetrain.setControl(drive.withVelocityX(xOutput*m_speed_modifier).withVelocityY(yOutput*m_speed_modifier).withRotationalRate(thetaOutput));
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
