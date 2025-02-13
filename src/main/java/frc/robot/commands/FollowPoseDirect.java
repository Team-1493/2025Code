// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.AprilTagCam;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.VisionConstants;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowPoseDirect extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CommandSwerveDrivetrain sd;
    boolean hasFirstTarget=false;
    boolean hasTarget=false;
    PhotonTrackedTarget target,lastTarget;
    Pose2d robotPose2d;
    Pose3d robotPose3d;
    Pose3d cameraPose = new Pose3d();
    Pose2d goalPose= new Pose2d();
    Transform3d  targetTransform = new Transform3d();
    Transform3d targetToGoal = new Transform3d();
    Transform3d ROBOT_TO_CAMERA = new Transform3d();
    public Pose3d targetPose3d     = new Pose3d();
    public Pose2d targetPose2d     = new Pose2d();
    ProfiledPIDController pidx, pidy, pidr; 

  public FollowPoseDirect(CommandSwerveDrivetrain m_sd) { 
      sd = m_sd;
      addRequirements(sd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasFirstTarget=false;
    hasTarget=false;
    robotPose2d=sd.getState().Pose;
    updateControllers();
    pidr.reset(robotPose2d.getRotation().getRadians());
    pidx.reset(robotPose2d.getX());
    pidy.reset(robotPose2d.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* 

    if (hasFirstTarget){

      if (hasTarget){
        robotPose2d = sd.getPose(); 
        robotPose3d= new Pose3d(
          robotPose2d.getX(),
          robotPose2d.getY(), 0, 
          new Rotation3d(0,0,robotPose2d.getRotation().getRadians())) ;
        if(AprilTagCam.BACK)ROBOT_TO_CAMERA=VisionConstants.ROBOT_TO_CAMERA_B;
        else ROBOT_TO_CAMERA=VisionConstants.ROBOT_TO_CAMERA_F;           
        cameraPose=robotPose3d.transformBy(VisionConstants.ROBOT_TO_CAMERA_B); 
       targetPose3d=cameraPose.transformBy(targetTransform);        
        goalPose=targetPose3d.transformBy(targetToGoal).toPose2d();


        pidx.setGoal(goalPose.getX());
        pidy.setGoal(goalPose.getY());
        pidr.setGoal(goalPose.getRotation().getRadians());
      }

      robotPose2d=sd.getPose();
      double dx = pidx.calculate(robotPose2d.getX());
      double dy = pidy.calculate(robotPose2d.getY());
      double dr = pidr.calculate(robotPose2d.getRotation().getRadians());

      sd.driveFieldCentric(dx, dy, dr);

      SmartDashboard.putNumber("FollowPose RobotPose x", robotPose2d.getX());
      SmartDashboard.putNumber("FollowPose RobotPose y", robotPose2d.getY());
      SmartDashboard.putNumber("FollowPose RobotPose rot", robotPose2d.getRotation().getDegrees());
      SmartDashboard.putNumber("FollowPose GoalPose x", goalPose.getX());
      SmartDashboard.putNumber("FollowPose GoalPose y", goalPose.getY());
      SmartDashboard.putNumber("FollowPose GoalPose rot", goalPose.getRotation().getDegrees());
      SmartDashboard.putNumber("FollowPose TargetTrans x", targetTransform.getX());
      SmartDashboard.putNumber("FollowPose TargetTrans y", targetTransform.getY());
      SmartDashboard.putNumber("FollowPose TargetTrans rot", Math.toDegrees(targetTransform.getRotation().getAngle()));
    }
    // System.out.println(AprilTagCam.visionPose.getX()+","+AprilTagCam.visionPose.getY());
      SmartDashboard.putBoolean("FollowPose hasTarget",hasTarget);

      */
  }

  @Override
  public boolean isFinished() {
    return (false);
  }

  public void updateControllers(){
      SmartDashboard.putNumber("FollowPose RobotPose x", 0);
      SmartDashboard.putNumber("FollowPose RobotPose y", 0);
      SmartDashboard.putNumber("FollowPose RobotPose rot",0);
      SmartDashboard.putNumber("FollowPose GoalPose x", 0);
      SmartDashboard.putNumber("FollowPose GoalPose y", 0);
      SmartDashboard.putNumber("FollowPose GoalPose rot",0);
      SmartDashboard.putNumber("FollowPose TargetTrans x",0);
      SmartDashboard.putNumber("FollowPose TargetTrans y", 0);
      SmartDashboard.putNumber("FollowPose TargetTrans rot",0);


      pidx = new ProfiledPIDController(
             Constants.followPose_Trans_kP,
             0,
             Constants.followPose_Trans_kD, 
             new TrapezoidProfile.Constraints(
            Constants.followPose_Trans_maxV,
            Constants.followPose_Trans_maxA));
      
      pidy = new ProfiledPIDController(
             Constants.followPose_Trans_kP,
             0,
             Constants.followPose_Trans_kD, 
             new TrapezoidProfile.Constraints(
            Constants.followPose_Trans_maxV,
            Constants.followPose_Trans_maxA));

      pidr = new ProfiledPIDController(
             Constants.followPose_Rot_kP,
             0,
             Constants.followPose_Rot_kD, 
             new TrapezoidProfile.Constraints(
            Constants.followPose_Rot_maxV,
            Constants.followPose_Rot_maxA));
      
       pidr.enableContinuousInput(-Math.PI, Math.PI);


  } 

}
