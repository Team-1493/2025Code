// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeReverse extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw claw;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public IntakeReverse(Claw m_claw) {
    claw=m_claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    claw.StopRollers();  
    claw.rearRollerFor();
    claw.chuteRollerRunReverse();


    

  }

  @Override
  public void execute() {
//    if (claw.encPosition<0.22 && claw.encPosition>-0.1 &&!elevFlag) {




  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
