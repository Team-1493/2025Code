// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utilities.VisionConstants;
import frc.robot.subsystems.LED;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static Robot instance;

  private final RobotContainer m_robotContainer;



  public Robot() {
    instance = this;

    m_robotContainer = new RobotContainer();
  }




  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {LED.enabled=false;}

  @Override
  public void disabledPeriodic() {
    LED.enabled=false;}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
//    VisionConstants.getAlliance();
//    VisionConstants.setColor();
    LED.enabled=true;
    m_robotContainer.releaseRamp().schedule();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {}                     
  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    LED.enabled=true;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

      m_robotContainer.releaseRamp().schedule();

  }

  @Override
  public void teleopPeriodic() {}

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
}
