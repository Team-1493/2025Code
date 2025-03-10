package frc.robot.commands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSystem;

public class DriveToCommands {
public  Command driveReef_FML,driveReef_FMR,
          driveReef_FLR,driveReef_FLL,
          driveReef_FRL,driveReef_FRR,
          driveReef_BML,driveReef_BMR,
          driveReef_BLL,driveReef_BLR,
          driveReef_BRL,driveReef_BRR,
          driveIntake_R, driveIntake_L;
  Pose2d targetPose;
  double reefOffsetX = VisionSystem.reefOffsetX;
  double reefOffsetY = VisionSystem.reefOffsetY;
  double intakeOffsetX = VisionSystem.intakeOffsetX;
  double intakeOffsetY = VisionSystem.intakeOffsetY;

  PathConstraints constraints = new PathConstraints(
      1.5, 
      1.5,
      Units.degreesToRadians(360),
      Units.degreesToRadians(450));

  Rotation2d rotZero=new Rotation2d(0);


  public DriveToCommands(){

// Front-Back    Right-Left Face    Right-Left Position on face

      if (VisionConstants.blue) driveReef_FML=getCommandLeft(18);
      else driveReef_FML=getCommandLeft(7);
 

      if (VisionConstants.blue) driveReef_FLL=getCommandLeft(19);
      else driveReef_FLL=getCommandLeft(6);

      if (VisionConstants.blue) driveReef_FRL=getCommandLeft(17);
      else driveReef_FRL=getCommandLeft(8);

      if (VisionConstants.blue) driveReef_BML=getCommandLeft(21);
      else driveReef_BML=getCommandLeft(10);

      if (VisionConstants.blue) driveReef_BLL=getCommandLeft(20);
      else driveReef_BLL=getCommandLeft(11);

      if (VisionConstants.blue) driveReef_BRL=getCommandLeft(22);
      else driveReef_BRL=getCommandLeft(9);


      
      if (VisionConstants.blue) driveReef_FMR=getCommandRight(18);
      else driveReef_FMR=getCommandRight(7);

      if (VisionConstants.blue) driveReef_FLR=getCommandRight(19);
      else driveReef_FLR=getCommandRight(6);

      if (VisionConstants.blue) driveReef_FRR=getCommandRight(17);
      else driveReef_FRR=getCommandRight(8);

      if (VisionConstants.blue) driveReef_BMR=getCommandRight(21);
      else driveReef_BMR=getCommandRight(10);

      if (VisionConstants.blue) driveReef_BLR=getCommandRight(20);
      else driveReef_BLR=getCommandRight(11);

      if (VisionConstants.blue) driveReef_BRR=getCommandRight(22);
      else driveReef_BRR=getCommandRight(9);

      if (VisionConstants.blue) driveIntake_R=getIntakeCommandRight(12);
      else driveIntake_R=getIntakeCommandRight(2);

      if (VisionConstants.blue) driveIntake_L=getIntakeCommandLeft(13);
      else driveIntake_L=getIntakeCommandLeft(1);



}

private Command getCommandLeft(int id){
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;
  targetPose = new Pose2d(
    targetPose.getX()+reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
    targetPose.getY()-reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
    new Rotation2d(rotRobot));

      Command drivePath= AutoBuilder.pathfindToPose(
        targetPose,constraints, 0.0);
      
    double dist = distanceToTarget();


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
  
    Command drivePath= AutoBuilder.pathfindToPose(
      targetPose,constraints, 0.0);

    double dist = distanceToTarget();

  return (drivePath);
}


private Command getIntakeCommandLeft(int id){
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;
  targetPose = new Pose2d(
      targetPose.getX()+intakeOffsetX*Math.sin(rotTarget)+intakeOffsetY*Math.cos(rotTarget),
      targetPose.getY()-intakeOffsetX*Math.cos(rotTarget)+intakeOffsetY*Math.sin(rotTarget),
      new Rotation2d(rotRobot));


  
      Command drivePath= AutoBuilder.pathfindToPose(
        targetPose,constraints, 0.0);


  return (drivePath);
}


private Command getIntakeCommandRight(int id){
  int index = id-1;
  targetPose = VisionConstants.aprilTagList.get(index).pose.toPose2d();
  double rotTarget = targetPose.getRotation().getRadians();
  double rotRobot=rotTarget+Math.PI;
  targetPose = new Pose2d(
      targetPose.getX()+intakeOffsetX*Math.sin(rotTarget)+intakeOffsetY*Math.cos(rotTarget),
      targetPose.getY()-intakeOffsetX*Math.cos(rotTarget)+intakeOffsetY*Math.sin(rotTarget),
      new Rotation2d(rotRobot));
  
      Command drivePath= AutoBuilder.pathfindToPose(
        targetPose,constraints, 1.0);


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