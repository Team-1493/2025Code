// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeAlgae2 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Claw claw;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public IntakeAlgae2(Elevator m_elevator,Claw m_claw) {
    claw=m_claw;
    elevator=m_elevator;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    elevator.toPosition(elevator.positionAlgae2);
    claw.toPosition(claw.positionAlgae2);
    claw.frontRollerRev();

  }

  @Override
  public void execute() {
     
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted) claw.frontRollerStop(); 
    else claw.holdAlgae();
  }

  @Override
  public boolean isFinished() {
    return claw.hasAlgae;
  }
}
