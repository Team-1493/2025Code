// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SpitCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw claw;
  int i;
  /**
   * @param subsystem The subsystem used by this command.
   */
  public SpitCoral(Claw m_claw) {
    claw=m_claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    claw.spitCoral();
    i=0;

  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    claw.stopRollers(); 
      }

  @Override
  public boolean isFinished() {
    if(!claw.hasCoral)i++;
    else i=0;
    return i>10;
  }
}
