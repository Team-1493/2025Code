package frc.robot.commands;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSystem;

public class DriveToCommands {
  Pose2d targetPose;
  double reefOffsetX = VisionSystem.reefOffsetX;
  double reefOffsetY = VisionSystem.reefOffsetY;
  double intakeOffsetX = VisionSystem.intakeOffsetX;
  double intakeOffsetY = VisionSystem.intakeOffsetY;

  PathConstraints constraints = new PathConstraints(
      2.5, 
      4,
      Units.degreesToRadians(360),
      Units.degreesToRadians(450));

  Rotation2d rotZero=new Rotation2d(0);
  CommandSwerveDrivetrain sd;

  public DriveToCommands(CommandSwerveDrivetrain m_sd){
      sd = m_sd;
}

public Command getCommandLeft(int id){
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;

  

  targetPose = new Pose2d(
    targetPose.getX()+reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
    targetPose.getY()-reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
    new Rotation2d(rotRobot));

  Pose2d tartgetPose2 = new Pose2d(
    targetPose.getX()+(reefOffsetX)*Math.sin(rotTarget)+(reefOffsetY+.5)*Math.cos(rotTarget),
    targetPose.getY()-(reefOffsetX)*Math.cos(rotTarget)+(reefOffsetY+.5)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));


List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        tartgetPose2,targetPose);
    
PathPlannerPath path2 = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
);


  double dist = distanceToTarget();
  Command drivePath;

  if(dist>0.6)

//      drivePath= AutoBuilder.pathfindToPose(
//          targetPose,constraints, 0.0);
  
  drivePath=AutoBuilder.pathfindThenFollowPath(path2, constraints);

  else {drivePath=new FollowPoseDirect(sd, targetPose);
      System.out.println("******************************   ");         
      System.out.println("******************************   FPD");
      System.out.println("******************************   ");            
      }

  return (drivePath);
}


public Command getCommandRight(int id){
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;
  targetPose = new Pose2d(
    targetPose.getX()-reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
    targetPose.getY()+reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
    new Rotation2d(rotRobot));

    double dist = distanceToTarget();
    Command drivePath;
  
    if(dist>0.6)
  
        drivePath= AutoBuilder.pathfindToPose(
            targetPose,constraints, 0.0);
    
    else {drivePath=new FollowPoseDirect(sd, targetPose);
      System.out.println("******************************   ");         
      System.out.println("******************************   FPD");
      System.out.println("******************************   ");            
    }
    return (drivePath);
}


public Command getIntakeCommand(int id){
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  targetPose = new Pose2d(
      targetPose.getX()+intakeOffsetX*Math.sin(rotTarget)+intakeOffsetY*Math.cos(rotTarget),
      targetPose.getY()-intakeOffsetX*Math.cos(rotTarget)+intakeOffsetY*Math.sin(rotTarget),
      new Rotation2d(rotTarget));

      double dist = distanceToTarget();
      Command drivePath;
    
      if(dist>0.6)

          drivePath= AutoBuilder.pathfindToPose(
              targetPose,constraints, 0.0);
      
      else {drivePath=new FollowPoseDirect(sd, targetPose);
        System.out.println("******************************   ");         
        System.out.println("******************************   FPD");
        System.out.println("******************************   ");            
      }
      return (drivePath);
}




private double distanceToTarget(){
  Pose2d robotPose = CommandSwerveDrivetrain.robotpose;

  double distance=
      Math.hypot(targetPose.getX()-robotPose.getX(), 
      targetPose.getY()-robotPose.getY());


  System.out.println("******************************   ");         
  System.out.println("******************************   "+distance);
  System.out.println("******************************   ");      
  return distance;
}


}
