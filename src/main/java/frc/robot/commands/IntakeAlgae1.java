// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeAlgae1 extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw claw;
  private Elevator elevator;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public IntakeAlgae1(Claw m_claw) {
    claw=m_claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    claw.toPosition(claw.positionAlgae1);
    claw.rollersRun(-1, 1);

  }

  @Override
  public void execute() {
     
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
