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
  boolean elevFlag=false,clawFlag=false,clawFlag2=false,clawFlag3=false;
  int i=0;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToNet(Elevator m_elevator,Claw m_claw) {
    claw=m_claw;
    elevator=m_elevator;
    elevFlag=false;
    clawFlag=false;
    clawFlag2=false;
    clawFlag3=false;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    elevFlag=false;clawFlag=false;
//    elevator.stopElevator();
//    claw.stopClaw();
    elevFlag=false;
    clawFlag=false;
    clawFlag2=false;
    claw.toPosition(claw.positionAlgae1);
    i=0;



    //claw.toPosition(claw.positionIntake);
    

  }

  @Override
  public void execute() {
//    System.out.println(elevator.elevatorPos+"   "+claw.encPosition +"   " +elevFlag+"    "+clawFlag+"   "+clawFlag2 );

    if (Math.abs(claw.encPosition-claw.positionAlgae1)<0.025 && !elevFlag) {
      elevator.toPosition(elevator.positionNet);
      elevFlag=true;}

    if (Math.abs(elevator.elevatorPos-elevator.positionNet)<.3 && elevFlag && !clawFlag){
        clawFlag=true;
        claw.toPosition(claw.positionNet);}

      if(Math.abs(claw.encPosition-claw.positionNet)<0.1 && elevFlag && clawFlag && !clawFlag2){
        claw.spitAlgae();
        clawFlag2=true;}

      if(clawFlag2) i++;
      if(i>10 && !clawFlag3) {
        clawFlag3=true;
//        claw.spitReverseAlgae();
      }
      
      

    }  
    



  

  @Override
  public void end(boolean interrupted) {
    claw.stopRollers();
  }

  @Override
  public boolean isFinished() {
  //  return (clawFlag&& elevFlag); 
  return(i>200);
  }

}

