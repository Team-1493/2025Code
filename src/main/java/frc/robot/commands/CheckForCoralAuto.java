// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CheckForCoralAuto extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw claw;
  private boolean done=false;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public CheckForCoralAuto(Claw m_claw) {
    claw=m_claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    done=false;

  }

  @Override
  public void execute() {
    if(!claw.coralSensor.get()){
        claw.stopRollers();
        done=true;

    }
    } 

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
      return (done);
  }
}
