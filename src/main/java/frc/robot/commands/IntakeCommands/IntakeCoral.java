// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Claw claw;
  boolean elevFlag=false;
  boolean clawFlag=false;
  int i = 0;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCoral(Elevator m_elevator,Claw m_claw) {
    claw=m_claw;
    elevator=m_elevator;
    elevFlag=false;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    claw.StopRollers();  
    elevFlag=false;
    clawFlag=false;
//    if(elevator.elevatorPos>0.15 && claw.encPosition>.15) claw.toPosition(.15);
    claw.toPosition(.15);
claw.rearRollerRev();
i=0;


    

  }

  @Override
  public void execute() {
//    if (claw.encPosition<0.22 && claw.encPosition>-0.1 &&!elevFlag) {

  if (Math.abs(claw.encPosition-claw.positionNeutral)<0.025) {
      elevator.toPosition(elevator.positionIntake);
      elevFlag=true;}   
    if (elevator.elevatorPos<0.25){
      claw.toPosition(claw.positionIntake);
      clawFlag=true;
    } 
    i++;



  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
//    return ((clawFlag && elevFlag) || i>150 );
    return (clawFlag && elevFlag);

  }
}
