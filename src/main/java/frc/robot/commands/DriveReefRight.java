// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Utilities.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveReefRight extends Command {
    private PathConstraints constraints;
    double rotTarget,rotRobot;
    Pose2d targetPose;
    private CommandSwerveDrivetrain sd;
    private Command drivePath;
    VisionConstants vc = new VisionConstants();
    
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveReefRight(CommandSwerveDrivetrain m_sd) {
        sd=m_sd;

        constraints = new PathConstraints(
            1.25, 
         2.5,
         Units.degreesToRadians(360),
         Units.degreesToRadians(450));

    addRequirements(sd);
  }

  @Override
  public void initialize() {
            
      double reefOffsetX = VisionSystem.reefOffsetX;
      double reefOffsetY = VisionSystem.reefOffsetY;


      sd.resetRotation(new Rotation2d(sd.headingTrue));
            
      Pose2d robotPose = sd.getPose();
      double xr=robotPose.getX();
      double yr=robotPose.getY();
      int id;

       
      // coord reef center 4.508,4.055
      xr=xr - 4.058;
      yr=yr - 4.055;  

      id=13;
      if (yr>xr/2 && yr<-xr/2) id=18;
      if (xr<0 && yr<xr/2) id = 17;
      if (xr<0 && yr>-xr/2 ) id = 19;
      if (xr>0 && yr>xr/2) id = 20;    
      if (yr<xr/2 && yr>-xr/2) id = 21;
      if (xr>0 && yr<-xr/2 ) id = 22;

      SmartDashboard.putNumber("Target ID", id);
      int index = id-1;
      targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
      rotTarget = targetPose.getRotation().getRadians();
      rotRobot=rotTarget+Math.PI;
      
      targetPose = new Pose2d(
          targetPose.getX()-reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
          targetPose.getY()+reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
          new Rotation2d(rotRobot));




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
