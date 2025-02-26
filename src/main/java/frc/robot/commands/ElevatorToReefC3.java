// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorToReefC3 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Claw claw;
  boolean elevFlag=false;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToReefC3(Elevator m_elevator,Claw m_claw) {
    claw=m_claw;
    elevator=m_elevator;
    elevFlag=false;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    elevFlag=false;
    claw.toPosition(claw.positionNeutral);



    //claw.toPosition(claw.positionIntake);
    

  }

  @Override
  public void execute() {
    
    if (Math.abs(claw.encPosition-claw.positionNeutral)<0.05 && !elevFlag) {
      elevator.toPosition(elevator.positionCoral3);
      elevFlag=true;}



  }

  @Override
  public void end(boolean interrupted) {
    claw.toPosition(claw.positionCoral3);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(elevator.elevatorPos-elevator.positionCoral3)<1 ); //claw.hasCoral;
  }
}
