// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class BumpIntoWall extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain sd;
  private int i;  

  /**
   * @param subsystem The subsystem used by this command.
   */
  public BumpIntoWall(CommandSwerveDrivetrain m_sd) {
    sd=m_sd;
    addRequirements(sd);
  }

  @Override
  public void initialize() {
    sd.driveRobotCentric(.25, 0, 0);
    i=0;
  }

  @Override
  public void execute() {
    if(sd.getChassisSpeed().vxMetersPerSecond>0.01) i++;
    else i=0; 
  }

  @Override
  public void end(boolean interrupted) {
    sd.stop();
  }

  @Override
  public boolean isFinished() {
    return sd.getChassisSpeed().vxMetersPerSecond<0.01 &&
    i>4;
  }
}
