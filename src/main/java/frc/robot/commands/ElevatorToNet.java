// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorToNet extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Claw claw;
  boolean elevFlag=false,clawFlag=false;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToNet(Elevator m_elevator,Claw m_claw) {
    claw=m_claw;
    elevator=m_elevator;
    elevFlag=false;
    clawFlag=false;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    elevFlag=false;clawFlag=false;
    elevator.stopElevator();
    claw.stopClaw();
    elevFlag=false;
    claw.toPosition(claw.positionNet);



    //claw.toPosition(claw.positionIntake);
    

  }

  @Override
  public void execute() {
    
    if (Math.abs(claw.encPosition-claw.positionNet)<0.03 && !elevFlag) {
      elevator.toPosition(elevator.positionNet);
      elevFlag=true;}

    if (Math.abs(elevator.elevatorPos-elevator.positionNet)<.4 && !clawFlag){
      clawFlag=true;
      claw.toPosition(claw.positionNet);
    }  



  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return (clawFlag&& elevFlag); 
  }
}
