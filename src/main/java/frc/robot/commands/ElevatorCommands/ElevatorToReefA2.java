// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorToReefA2 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Claw claw;
  boolean elevFlag=false;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToReefA2(Elevator m_elevator,Claw m_claw) {
    claw=m_claw;
    elevator=m_elevator;
    elevFlag=false;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    elevFlag=false;
    claw.rollersRun(-4, 4);
    claw.toPosition(claw.positionAlgae2);


    //claw.toPosition(claw.positionIntake);
    

  }

  @Override
  public void execute() {
    
    if (Math.abs(claw.encPosition-claw.positionAlgae2)<0.025 && !elevFlag) {
      elevator.toPosition(elevator.positionAlgae2);
      elevFlag=true;}



  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) claw.stopRollers();
    else claw.holdAlgae();
  }

  @Override
  public boolean isFinished() {
    return (claw.hasAlgae);
  }
}
