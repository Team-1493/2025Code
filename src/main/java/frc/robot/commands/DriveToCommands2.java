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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utilities.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.VisionSystem;

public class DriveToCommands2 {
  Pose2d targetPose,endPose;
  double reefOffsetXLeft = VisionSystem.reefOffsetXLeft+.075;
  double reefOffsetXRight = VisionSystem.reefOffsetXRight+.075;
  double reefOffsetY = VisionSystem.reefOffsetY;
  double intakeOffsetX = VisionSystem.intakeOffsetX;
  double intakeOffsetY = VisionSystem.intakeOffsetY;
  double distThreshold=0.51;

  PathConstraints constraints = new PathConstraints(
      1.75, 
      2,
      Units.degreesToRadians(360),
      Units.degreesToRadians(450));

  Rotation2d rotZero=new Rotation2d(0);
  CommandSwerveDrivetrain sd;
            public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
            public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);

  public DriveToCommands2(CommandSwerveDrivetrain m_sd){
      sd = m_sd;
      SmartDashboard.putNumber("Net X", 0);
      SmartDashboard.putNumber("Net Y", 0);
      SmartDashboard.putNumber("Net Lead X", 5);

}

public Command getCommandLeft(){

  int id = getID();
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;

  endPose = new Pose2d(
    targetPose.getX()+ reefOffsetXLeft*Math.sin(rotTarget)+(reefOffsetY - 0.15)*Math.cos(rotTarget),
    targetPose.getY() - reefOffsetXLeft*Math.cos(rotTarget)+(reefOffsetY - 0.15)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));


  targetPose = new Pose2d(
    targetPose.getX()+reefOffsetXLeft*Math.sin(rotTarget)+(reefOffsetY+ 0.5)*Math.cos(rotTarget),
    targetPose.getY()-reefOffsetXLeft*Math.cos(rotTarget)+(reefOffsetY+ 0.5)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));
  


    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(sd.getPose().getTranslation(), getPathVelocityHeading(sd.getFieldVelocity(), targetPose)),
      targetPose
  );

    IdealStartingState iss = new IdealStartingState(
          getVelocityMagnitude(sd.getFieldVelocity()), sd.getPose().getRotation());

          PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,iss, 
            new GoalEndState(1, targetPose.getRotation())
        );

        path.preventFlipping = true;

        double dist = distanceToTarget();
        if(dist>distThreshold)


        return (
          AutoBuilder.followPath(path)).andThen(
            new InstantCommand(() -> LED.nearTarget=true),
            PositionPIDCommand.generateCommand(sd, endPose, (
                DriverStation.isAutonomous() ? kAutoAlignAdjustTimeout : kTeleopAlignAdjustTimeout
            )
         )
         .finallyDo(() -> {
                sd.stop();
        }));

        else return( PositionPIDCommand.generateCommand(sd, endPose, (
          DriverStation.isAutonomous() ? kAutoAlignAdjustTimeout : kTeleopAlignAdjustTimeout
           ))); 


}


public Command getCommandRight(){

  int id = getID();
  int index = id-1;

  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;

  endPose = new Pose2d(
    targetPose.getX()-reefOffsetXRight*Math.sin(rotTarget)+(reefOffsetY - 0.15)*Math.cos(rotTarget),
    targetPose.getY()+reefOffsetXRight*Math.cos(rotTarget)+(reefOffsetY - 0.15)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));


  targetPose = new Pose2d(
    targetPose.getX()-reefOffsetXRight*Math.sin(rotTarget)+(reefOffsetY+ .5)*Math.cos(rotTarget),
    targetPose.getY()+reefOffsetXRight*Math.cos(rotTarget)+(reefOffsetY+ 0.5)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));

    double dist = distanceToTarget();    

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(sd.getPose().getTranslation(), getPathVelocityHeading(sd.getFieldVelocity(), targetPose)),
      targetPose
  );

    IdealStartingState iss = new IdealStartingState(
          getVelocityMagnitude(sd.getFieldVelocity()), sd.getPose().getRotation());

          PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,iss, 
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

}


public Command getCommandCenter(){

  int id = getID();
  int index = id-1;

  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;

  endPose = new Pose2d(
    targetPose.getX()+(reefOffsetY-0.15)*Math.cos(rotTarget),
    targetPose.getY()+(reefOffsetY-0.15)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));


  targetPose = new Pose2d(
    targetPose.getX()+(reefOffsetY+ 0.5)*Math.cos(rotTarget),
    targetPose.getY()+(reefOffsetY+ 0.5)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));

    double dist = distanceToTarget();    

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(sd.getPose().getTranslation(), getPathVelocityHeading(sd.getFieldVelocity(), targetPose)),
      targetPose
  );

    IdealStartingState iss = new IdealStartingState(
          getVelocityMagnitude(sd.getFieldVelocity()), sd.getPose().getRotation());

          PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,iss, 
            new GoalEndState(1, targetPose.getRotation())
        );

        path.preventFlipping = true;

        return (AutoBuilder.followPath(path).andThen(
            PositionPIDCommand.generateCommand(sd, endPose, (
                DriverStation.isAutonomous() ? kAutoAlignAdjustTimeout : kTeleopAlignAdjustTimeout
            ))
         )).finallyDo((interupt) -> {
            if (interupt) { //if this is false then the position pid would've X braked & called the same method
                sd.stop();
            }
        });

}



public Command getIntakeCommand(){

  boolean blue = VisionConstants.blue;
  int id = 13;
  double yr = sd.getPose().getY();
  if (yr>4.055) id = blue ? 13 : 1;
  else id = blue ? 12 : 2;

  int index = id-1;
  double intakeOffsetLead = 0;

  intakeOffsetX=0;
  intakeOffsetY=0.35;
  intakeOffsetLead =0.43;


  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget;

  endPose = new Pose2d(
    targetPose.getX()+(intakeOffsetY)*Math.cos(rotTarget),
    targetPose.getY()+(intakeOffsetY)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));


  targetPose = new Pose2d(
    targetPose.getX()+(intakeOffsetY+ intakeOffsetLead)*Math.cos(rotTarget),
    targetPose.getY()+(intakeOffsetY+ intakeOffsetLead)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));

    double dist = distanceToTarget();    

    Waypoint wp = new Waypoint(targetPose.getTranslation(),endPose.getTranslation(),null);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(sd.getPose().getTranslation(), getPathVelocityHeading(sd.getFieldVelocity(), targetPose)),
      endPose
  );

  waypoints.remove(1);
  waypoints.add(wp);

    IdealStartingState iss = new IdealStartingState(
          getVelocityMagnitude(sd.getFieldVelocity()), sd.getPose().getRotation());

          PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,iss, 
            new GoalEndState(0, targetPose.getRotation())
        );

        path.preventFlipping = true;
/* 
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

*/

        
        return (AutoBuilder.followPath(path).
        andThen(sd.Stop()));

}



public Command getNetCommand(){

  boolean blue = VisionConstants.blue;
  double yr = sd.getPose().getY();
  double ytarget,xtarget,netLeadX;

  ytarget=SmartDashboard.getNumber("Net Y", 5);
  xtarget=SmartDashboard.getNumber("Net X", 5);
  netLeadX = SmartDashboard.getNumber("Net Lead X", 5);



  targetPose = new Pose2d(xtarget,ytarget,new Rotation2d(0));
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget;

  endPose = targetPose;


  targetPose = new Pose2d(
    targetPose.getX()- netLeadX,
    targetPose.getY(),
    new Rotation2d(rotRobot));

    double dist = distanceToTarget();    

    Waypoint wp = new Waypoint(targetPose.getTranslation(),endPose.getTranslation(),null);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(sd.getPose().getTranslation(), getPathVelocityHeading(sd.getFieldVelocity(), targetPose)),
      endPose
  );

  waypoints.remove(1);
  waypoints.add(wp);

    IdealStartingState iss = new IdealStartingState(
          getVelocityMagnitude(sd.getFieldVelocity()), sd.getPose().getRotation());

          PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,iss, 
            new GoalEndState(0, targetPose.getRotation())
        );

        path.preventFlipping = true;
/* 
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

*/

        
        return (AutoBuilder.followPath(path).
        andThen(sd.Stop()));

}






public Command getProcessorCommand(){

  boolean blue = VisionConstants.blue;
  int id = 16;
  if (blue) id=16;
  else id = 3;

  int index = id-1;

  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;

  endPose = new Pose2d(
    targetPose.getX()+(reefOffsetY)*Math.cos(rotTarget),
    targetPose.getY()+(reefOffsetY)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));


  targetPose = new Pose2d(
    targetPose.getX()+(reefOffsetY+ 0.45)*Math.cos(rotTarget),
    targetPose.getY()+(reefOffsetY+ 0.45)*Math.sin(rotTarget),
    new Rotation2d(rotRobot));

    double dist = distanceToTarget();    

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(sd.getPose().getTranslation(), getPathVelocityHeading(sd.getFieldVelocity(), targetPose)),
      targetPose
  );

    IdealStartingState iss = new IdealStartingState(
          getVelocityMagnitude(sd.getFieldVelocity()), sd.getPose().getRotation());

          PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,iss, 
            new GoalEndState(1, targetPose.getRotation())
        );

        path.preventFlipping = true;

        return (
          new InstantCommand(() -> LED.enabled=true).andThen(
        AutoBuilder.followPath(path)).andThen(
            PositionPIDCommand.generateCommand(sd, endPose, (
                DriverStation.isAutonomous() ? kAutoAlignAdjustTimeout : kTeleopAlignAdjustTimeout
            ))
         )).finallyDo((interupt) -> {
                sd.stop();
            
        });

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
  boolean blue = VisionConstants.blue;
  Pose2d robotPose = sd.getPose();
  double xr=robotPose.getX();
  double yr=robotPose.getY();
  int id;

  // y = -1/2x  ,  y = 1/2 x  ,   x = 0
  // coord reef center 4.508,4.055
  xr=xr - 4.058;
  yr=yr - 4.055;  

  id=13;
  if (yr>xr/2 && yr<-xr/2) id = blue? 18:7;
  if (xr<0 && yr<xr/2) id = blue ? 17 : 8;
  if (xr<0 && yr>-xr/2 ) id = blue ? 19 : 6;
  if (xr>0 && yr>xr/2) id = blue ? 20 : 11;    
  if (yr<xr/2 && yr>-xr/2) id = blue ? 21 : 10;
  if (xr>0 && yr<-xr/2 ) id = blue ? 22 : 9 ;

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
