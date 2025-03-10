// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RearIntake;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ReleaseRamp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RearIntake intake;
  private VoltageOut  voltOut= new VoltageOut(1);
  

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ReleaseRamp(RearIntake m_intake) {
    intake=m_intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.motor.setControl(voltOut);
    RearIntake.releasedRamp=true;
  }

  @Override
  public void execute() {
    }


     
  

  @Override
  public void end(boolean interrupted) {
    intake.motor.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return (!intake.limitSwitch.get());
  }
}
