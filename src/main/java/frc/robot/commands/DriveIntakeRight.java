// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.VisionConstants;
import frc.robot.subsystems.ActionCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveIntakeRight extends Command {
    private PathConstraints constraints;
    double rotTarget,rotRobot;
    Pose2d targetPose;
    private  double reefOffsetX = 0,reefOffsetY=.75;
    private CommandSwerveDrivetrain sd;
    private Command drivePath;
    private int id;

    
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveIntakeRight(CommandSwerveDrivetrain m_sd) {
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
    
    double yr = sd.getPose().getY();
    if (yr>4.055) id = 13;

    else id = 12;
    

    targetPose = VisionConstants.AprilTagList.get(id-1).pose.toPose2d();
    rotTarget = targetPose.getRotation().getRadians();
    rotRobot=rotTarget+Math.PI;
    targetPose = new Pose2d(
        targetPose.getX()+reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
        targetPose.getY()-reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
        new Rotation2d(rotRobot+Math.PI));
            

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
    sd.stop();
    sd.zeroGyro(); 
  }

  @Override
    public boolean isFinished() {
        return drivePath.isFinished();
  }
}
