package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.VisionConstants;

public class ActionCommands {

    static private PathConstraints constraints;
    Pose2d targetPose;
    public static double reefOffsetX=.25,reefOffsetY=.75;
    static public Command driveReefRight,driveReefLeft;

public  ActionCommands(CommandSwerveDrivetrain sd,Elevator elevator,Claw claw){

    constraints = new PathConstraints(
        1.5, 
        1.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(450));



        // DriveReefRight
            targetPose = VisionConstants.AprilTagList.get(21).pose.toPose2d();
            double rotTarget = targetPose.getRotation().getRadians();
            double rotRobot=rotTarget+Math.PI;
            targetPose = new Pose2d(
                targetPose.getX()+reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
                targetPose.getY()-reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
                new Rotation2d(rotRobot));
            
//            targetPose = targetPose.plus(new Transform2d(1,0,new Rotation2d(Math.PI/2-rotTarget)));

                
                
            driveReefLeft= AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0).andThen
                (new InstantCommand(() ->sd.resetHeadingController(rotRobot)));






        }


}



