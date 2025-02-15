// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final Claw claw;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCoral(Elevator m_elevator,Claw m_claw) {
    claw=m_claw;
    elevator=m_elevator;
    addRequirements(claw,elevator);
  }

  @Override
  public void initialize() {
    elevator.toPosition(elevator.positionIntake);
    claw.toPosition(claw.positionIntake);
    claw.rearRollerFor();

  }

  @Override
  public void execute() {
    if (claw.hasCoral) claw.stopRollerRear(); 
  }

  @Override
  public void end(boolean interrupted) {
    claw.stopRollerRear();
    if (claw.hasCoral) {
        claw.toPosition(claw.positionIntake);
        elevator.toPosition(elevator.positionIntake);
    }
  }

  @Override
  public boolean isFinished() {
    return claw.hasCoral;
  }
}
