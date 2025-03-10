// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorToNet extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Claw claw;
  boolean elevFlag=false;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToNet(Elevator m_elevator,Claw m_claw) {
    claw=m_claw;
    elevator=m_elevator;
    elevFlag=false;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
//    elevator.stopElevator();
    elevFlag=false;
    elevator.toPosition(elevator.positionNet);



    //claw.toPosition(claw.positionIntake);
    

  }

  @Override
  public void execute() {
    
    if (Math.abs(elevator.elevatorPos-elevator.positionNet)<0.5 && !elevFlag) {
      elevFlag=true;}



  }

  @Override
  public void end(boolean interrupted) {
    claw.toPosition(claw.positionNet);
  }

  @Override
  public boolean isFinished() {
    return (elevFlag);
  }
}
