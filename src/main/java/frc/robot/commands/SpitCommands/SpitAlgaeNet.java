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

  /**
   * @param subsystem The subsystem used by this command.
   */
  public SpitAlgaeNet(Claw m_claw, Elevator m_elevator) {
    claw=m_claw;
    elevator=m_elevator;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    claw.toPosition(claw.positionNet);
    

  }

  @Override
  public void execute() {
    if(!elevatorFlag && claw.encPosition<.215){
        elevator.toPosition(elevator.positionNet);
        elevatorFlag=true;
    }
    if (Math.abs(elevator.elevatorPos-elevator.positionNet)<.03){
        claw.spitAlgae();
    }


     
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted) claw.stopRollers(); 
    else claw.holdAlgae();
  }

  @Override
  public boolean isFinished() {
    return claw.hasAlgae;
  }
}
