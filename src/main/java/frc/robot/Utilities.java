package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSystem;


public class Utilities {

    CommandSwerveDrivetrain sd;
    VisionSystem vs;
    VisionConstants vc = new VisionConstants();

    Utilities(CommandSwerveDrivetrain m_sd, VisionSystem m_vs){
        sd=m_sd;
        vs=m_vs;
    }

    public Pose2d getTagTargetPose(){
        Pose2d targetPose;
        double rotRobot;
        double rotTarget;

        double reefOffsetX = -VisionSystem.reefOffsetX;
        double reefOffsetY = VisionSystem.reefOffsetY;

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

         int index = id-1;
        targetPose = vc.aprilTagList.get(index).pose.toPose2d();
        System.out.println("***********************  "+targetPose.getX());
        System.out.println("***********************  "+targetPose.getY());
        rotTarget = targetPose.getRotation().getRadians();
        rotRobot=rotTarget+Math.PI;
        targetPose = new Pose2d(
            targetPose.getX()+reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
            targetPose.getY()-reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
            new Rotation2d(rotRobot));
        return (targetPose);
    }

    public Command getReefRight(){
        Command driveReefRight;
        PathConstraints constraints = new PathConstraints(
            1.5, 
         1.5,
         Units.degreesToRadians(360),
         Units.degreesToRadians(450));
        double rotTarget,rotRobot;
        Pose2d targetPose;
  double reefOffsetX = -VisionSystem.reefOffsetX;
            double reefOffsetY = VisionSystem.reefOffsetY;

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


//            int index=  VisionSystem.closestReefID-1;
             int index = id-1;
            targetPose = vc.aprilTagList.get(index).pose.toPose2d();
            rotTarget = targetPose.getRotation().getRadians();
            rotRobot=rotTarget+Math.PI;
            targetPose = new Pose2d(
                targetPose.getX()+reefOffsetX*Math.sin(rotTarget)+reefOffsetY*Math.cos(rotTarget),
                targetPose.getY()-reefOffsetX*Math.cos(rotTarget)+reefOffsetY*Math.sin(rotTarget),
                new Rotation2d(rotRobot));
 //           }

//          if(VisionSystem.hasReefTarget){
            sd.TurnOffHeadingControl().andThen(
            driveReefRight= AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0));

        return driveReefRight;
    }
}
