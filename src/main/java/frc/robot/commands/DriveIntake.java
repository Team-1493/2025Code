// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Utilities.VisionConstants;
import frc.robot.subsystems.ActionCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveIntake extends Command {
    private PathConstraints constraints;
    double rotTarget,rotRobot;
    Pose2d targetPose;
    private  double intakeOffsetX = VisionSystem.intakeOffsetX;
    private  double intakeOffsetY = VisionSystem.intakeOffsetX;
    private CommandSwerveDrivetrain sd;
    private Command drivePath;
    VisionConstants vc = new VisionConstants();

    

    
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveIntake(CommandSwerveDrivetrain m_sd) {
        sd=m_sd;

        constraints = new PathConstraints(
            1.5, 
         1.5,
         Units.degreesToRadians(360),
         Units.degreesToRadians(450));

    addRequirements(sd);
  }

  @Override
  public void initialize() {
    int id;
    double yr = sd.getPose().getY();
    if (yr>4.055) id = 13;

    else id = 12;
    

    targetPose = vc.aprilTagList.get(id-1).pose.toPose2d();
    rotTarget = targetPose.getRotation().getRadians();
    rotRobot=rotTarget;
    targetPose = new Pose2d(
        targetPose.getX()+intakeOffsetX*Math.sin(rotTarget)+intakeOffsetY*Math.cos(rotTarget),
        targetPose.getY()-intakeOffsetX*Math.cos(rotTarget)+intakeOffsetY*Math.sin(rotTarget),
        new Rotation2d(rotRobot));
            

    sd.turnOffHeadingControl();
    drivePath= AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0);

            drivePath.initialize();

            
  }

  @Override
  public void execute() {
    drivePath.execute();
  }

  @Override
  public void end(boolean interrupted) {
    drivePath.end(false);
    sd.stop();
    sd.zeroGyro();    

  }

  @Override
    public boolean isFinished() {
        return drivePath.isFinished();
  }
}
