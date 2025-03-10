// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ZeroElevator;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean enabled;

  private final RobotContainer m_robotContainer;

  public Robot() {

    m_robotContainer = new RobotContainer();
    PathfindingCommand.warmupCommand().schedule();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {enabled=false;}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    enabled=true;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand().alongWith(m_robotContainer.releaseRamp());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    else m_robotContainer.releaseRamp().schedule();;
  }

  @Override
  public void autonomousPeriodic() {}                     
  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    enabled=true;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
//    m_robotContainer.zeroGyro().schedule();;
//    m_robotContainer.zeroElevator().schedule();
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
