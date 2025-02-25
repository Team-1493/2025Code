// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ZeroElevator extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  boolean elevFlag=false;
  int i;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ZeroElevator(Elevator m_elevator) {
    elevator=m_elevator;
    elevFlag=false;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevFlag=false;
    i=0;
    elevator.manualDown();
}

  @Override
  public void execute() {
    i++;
    if (Math.abs(elevator.elevatorRight.getVelocity().getValueAsDouble())>0.001 )
        i=0;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
    elevator.elevatorRight.setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return (i>=10 && Math.abs(elevator.elevatorRight.getVelocity().getValueAsDouble())<0.001 );
  }
}
