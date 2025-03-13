// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ZeroElevator;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean enabled;
  public static Robot instance;

  private final RobotContainer m_robotContainer;
  private final AddressableLED m_led; 
  private final AddressableLEDBuffer m_ledBuffer;

  private LEDPattern currentPattern = null;
  private final LEDPattern m_blue = LEDPattern.solid(Color.kBlue);
  private final LEDPattern m_red = LEDPattern.solid(Color.kRed);
  private final LEDPattern m_green = LEDPattern.solid(Color.kGreen);
  private final LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
  private final LEDPattern m_violetred = LEDPattern.solid(Color.kPaleVioletRed);


  private double finishedAutoAlignAt = -1;

  public Robot() {
    instance = this;

    m_robotContainer = new RobotContainer();
    PathfindingCommand.warmupCommand().schedule();


    m_led = new AddressableLED(8);
    // CHANGE LENGTH IF NEEDED
    m_ledBuffer = new AddressableLEDBuffer(160);
    m_led.setLength(m_ledBuffer.getLength());
  }



  public void finishedAutoAlign() {
    finishedAutoAlignAt = Timer.getFPGATimestamp();
  }

  private void setPattern(LEDPattern pattern) {
    if (currentPattern == pattern) return;
    currentPattern = pattern;
    currentPattern.applyTo(m_ledBuffer);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void robotPeriodic() {
    double timeAfterAutoAlign = 4;
    if (Timer.getFPGATimestamp() < finishedAutoAlignAt + timeAfterAutoAlign) {
      setPattern(m_violetred);
    } else if (Timer.getMatchTime() < 10) { // less than ten seconds left
      setPattern(m_yellow);
    } else if (m_robotContainer.claw.hasCoral) {
      setPattern(m_green);
    } else {
      setPattern(m_blue);
    }
    
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
//    else m_robotContainer.releaseRamp().schedule();;
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
//      m_robotContainer.releaseRamp().schedule();

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
