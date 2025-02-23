// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorToReef extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Claw claw;
  boolean elevFlag=false;
  double pos;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorToReef(Elevator m_elevator,Claw m_claw,double m_pos) {
    claw=m_claw;
    elevator=m_elevator;
    pos=m_pos;
    elevFlag=false;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    elevFlag=false;
    claw.toPosition(claw.positionCoral1);



    //claw.toPosition(claw.positionIntake);
    

  }

  @Override
  public void execute() {
    if (claw.encPosition<0.215 && !elevFlag) {
      elevator.toPosition(pos);
      elevFlag=true;}   



  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return elevFlag; //claw.hasCoral;
  }
}
