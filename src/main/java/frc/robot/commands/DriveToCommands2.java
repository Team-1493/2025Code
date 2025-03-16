package frc.robot.commands;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Utilities.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSystem;

public class DriveToCommands2 {
  Pose2d targetPose,endPose;
  double reefOffsetX = VisionSystem.reefOffsetX;
  double reefOffsetY = VisionSystem.reefOffsetY;
  double intakeOffsetX = VisionSystem.intakeOffsetX;
  double intakeOffsetY = VisionSystem.intakeOffsetY;

  PathConstraints constraints = new PathConstraints(
      2, 
      2,
      Units.degreesToRadians(360),
      Units.degreesToRadians(450));

  Rotation2d rotZero=new Rotation2d(0);
  CommandSwerveDrivetrain sd;
            public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
            public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);

  public DriveToCommands2(CommandSwerveDrivetrain m_sd){
      sd = m_sd;
}

public Command getCommandLeft(){
  int id = getID();
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;

  targetPose = new Pose2d(
    targetPose.getX()+reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
    targetPose.getY()-reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
    new Rotation2d(rotRobot));

    endPose = new Pose2d(
      targetPose.getX()-reefOffsetX*Math.sin(rotTarget)+(reefOffsetY)*Math.cos(rotTarget),
      targetPose.getY()+reefOffsetX*Math.cos(rotTarget)+(reefOffsetY)*Math.sin(rotTarget),
      new Rotation2d(rotRobot));

  double dist = distanceToTarget();
  Command drivePath;

  if(dist<0)

//      drivePath= AutoBuilder.pathfindToPose(
//          targetPose,constraints, 0.0);
  
  drivePath=AutoBuilder.pathfindToPose(targetPose, constraints);

  else {drivePath=new FollowPoseDirect(sd, targetPose);
      System.out.println("******************************   ");         
      System.out.println("******************************   FPD");
      System.out.println("******************************   ");            
      }

  return (drivePath);
}


public Command getCommandRight(){
  int id = getID();
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;

  endPose = new Pose2d(
    targetPose.getX()-reefOffsetX*Math.sin(rotTarget)+(reefOffsetY)*Math.cos(rotTarget),
    targetPose.getY()+reefOffsetX*Math.cos(rotTarget)+(reefOffsetY)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));


  targetPose = new Pose2d(
    targetPose.getX()-reefOffsetX*Math.sin(rotTarget)+(reefOffsetY+.8)*Math.cos(rotTarget),
    targetPose.getY()+reefOffsetX*Math.cos(rotTarget)+(reefOffsetY+.8)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));


    double dist = distanceToTarget();
    Command drivePath;
  
    

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(sd.getPose().getTranslation(), getPathVelocityHeading(sd.getFieldVelocity(), targetPose)),
      targetPose
  );

    



    IdealStartingState iss = new IdealStartingState(
          getVelocityMagnitude(sd.getFieldVelocity()), sd.getPose().getRotation());

          PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,
            new IdealStartingState(getVelocityMagnitude(sd.getFieldVelocity()), sd.getPose().getRotation()), 
            new GoalEndState(1, targetPose.getRotation())
        );

        path.preventFlipping = true;

        return (AutoBuilder.followPath(path).andThen(
            Commands.print("start position PID loop"),
            PositionPIDCommand.generateCommand(sd, endPose, (
                DriverStation.isAutonomous() ? kAutoAlignAdjustTimeout : kTeleopAlignAdjustTimeout
            ))
         )).finallyDo((interupt) -> {
            if (interupt) { //if this is false then the position pid would've X braked & called the same method
                sd.stop();
            }
        });


/* 
    if(dist<0)
  
    drivePath=AutoBuilder.pathfindToPose(targetPose, constraints);
  
    else {drivePath=new FollowPoseDirect(sd, targetPose);
      System.out.println("******************************   ");         
      System.out.println("******************************   FPD");
      System.out.println("******************************   ");            
    }
    return (drivePath);
    */
}


public Command getIntakeCommand(){
  int id = 13;
  double yr = sd.getPose().getY();
  if (yr>4.055) id = 13;
  else id = 12;
  
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  targetPose = new Pose2d(
      targetPose.getX()+intakeOffsetX*Math.sin(rotTarget)+intakeOffsetY*Math.cos(rotTarget),
      targetPose.getY()-intakeOffsetX*Math.cos(rotTarget)+intakeOffsetY*Math.sin(rotTarget),
      new Rotation2d(rotTarget));

      double dist = distanceToTarget();
      Command drivePath;
    
      if(dist<0)

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

private int getID(){
  Pose2d robotPose = sd.getPose();
  double xr=robotPose.getX();
  double yr=robotPose.getY();
  int id;

  // y = -1/2x  ,  y = 1/2 x  ,   x = 0
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

  return id;

}


private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(sd.getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

   

}
