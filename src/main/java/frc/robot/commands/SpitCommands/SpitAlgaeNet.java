// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SpitCommands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SpitAlgaeNet extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw claw;
  private Elevator elevator;
  boolean elevatorFlag=false;
  int counter=0;
  /**
   * @param subsystem The subsystem used by this command.
   */
  public SpitAlgaeNet(Claw m_claw) {
    claw=m_claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    counter=0;
    claw.toPosition(claw.positionNet);
    claw.rollersRun(-3, 3);
    

  }

  @Override
  public void execute() {
    counter = counter + 1;
    if (counter==20)    claw.spitAlgae();
  


     
  }

  @Override
  public void end(boolean interrupted) {
  claw.stopRollers(); 
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
