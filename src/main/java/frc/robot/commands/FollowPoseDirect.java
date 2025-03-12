// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Utilities.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowPoseDirect extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CommandSwerveDrivetrain sd;
    Pose2d robotPose2d;
    Pose2d goalPose= new Pose2d();
    double finalRawRotation;
    double deltaRot;
    VisionConstants vc = new VisionConstants();
    private double followPose_Trans_kP=4;
    private double followPose_Trans_kD=.1;
    private double followPose_Rot_kP=4;
    private double followPose_Rot_kD=.5;
    private double followPose_Trans_maxV=2;
    private double followPose_Trans_maxA=8;
    private double followPose_Rot_maxV=1;
    private double followPose_Rot_maxA=2;



    public Pose2d targetPose2d     = new Pose2d();
    ProfiledPIDController pidx, pidy, pidr; 

  public FollowPoseDirect(CommandSwerveDrivetrain m_sd, Pose2d goalPose) { 
      sd = m_sd;
      addRequirements(sd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    robotPose2d=sd.getState().Pose;
    updateControllers();
    pidr.reset(robotPose2d.getRotation().getRadians());
    pidx.reset(robotPose2d.getX());
    pidy.reset(robotPose2d.getY());
//    deltaRot=sd.getPose().getRotation().getRadians()-sd.rawGyroInitial;
//    finalRawRotation=goalPose.getRotation().getRadians()-deltaRot;
//    goalPose=new Pose2d(goalPose.getX(),goalPose.getY(),new Rotation2d(finalRawRotation));
    pidx.setGoal(goalPose.getX());
    pidy.setGoal(goalPose.getY());
    pidr.setGoal(goalPose.getRotation().getRadians());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      

      robotPose2d=sd.getPose();
      double dx = pidx.calculate(robotPose2d.getX());
      double dy = pidy.calculate(robotPose2d.getY());
      double dr = pidr.calculate(robotPose2d.getRotation().getRadians());

      System.out.println(dy+"   "+robotPose2d.getY()+
          "    "+pidy.getPositionError());
      
      if (Math.abs(pidx.getPositionError())<.03) dx=0;    
      if (Math.abs(pidy.getPositionError())<.03) dy=0;    
      if (Math.abs(pidr.getPositionError())<.01) dr=0;    

      sd.driveRobotCentric(dx, dy, dr);

  }

  @Override
  public boolean isFinished() {
    return (false);
  }

  public void updateControllers(){


      pidx = new ProfiledPIDController(
             followPose_Trans_kP*.5,
             0,
             followPose_Trans_kD, 
             new TrapezoidProfile.Constraints(
            followPose_Trans_maxV,
            followPose_Trans_maxA)  );
      
      pidy = new ProfiledPIDController(
             followPose_Trans_kP,
             0,
             followPose_Trans_kD, 
             new TrapezoidProfile.Constraints(
            followPose_Trans_maxV,
            followPose_Trans_maxA));

      pidr = new ProfiledPIDController(
             followPose_Rot_kP,
             0,
             followPose_Rot_kD, 
             new TrapezoidProfile.Constraints(
            followPose_Rot_maxV,
            followPose_Rot_maxA));
      
       pidr.enableContinuousInput(-Math.PI, Math.PI);


  }
  
     private Pose2d getTagTargetPose(){
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

}
