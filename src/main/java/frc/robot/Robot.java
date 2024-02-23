// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
<<<<<<< Updated upstream
import frc.robot.subsystems.Limelight;
=======
import frc.robot.subsystems.LimeLight;
>>>>>>> Stashed changes

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Optional<Alliance> m_alliance = null;

  private void checkUpdateAlliance(){    
    Optional<Alliance> alliance =  DriverStation.getAlliance();
    if(DriverStation.isDSAttached() && alliance.isPresent()  && alliance != m_alliance){
      //Limelight frontLL = m_robotContainer.getFLL();
      Limelight backLL = m_robotContainer.getBLL();
      //frontLL.setAlliance(alliance);
      backLL.setAlliance(alliance.get());
    }
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    checkUpdateAlliance();
<<<<<<< Updated upstream
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
=======
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

>>>>>>> Stashed changes
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    checkUpdateAlliance();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    checkUpdateAlliance();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    checkUpdateAlliance();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  private void checkUpdateAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (DriverStation.isDSAttached() && alliance.isPresent()) {
      LimeLight backLL = m_robotContainer.getBackLimelight();
      backLL.setAlliance(alliance.get());
    }
  }
  
}

