package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AprilTagCam;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSystem;

public class FollowAprilTag extends FollowPoseDirect {
  public FollowAprilTag(CommandSwerveDrivetrain m_sd){
    this(m_sd, new Transform3d(-0.35,0,0,new Rotation3d(0,0,0)));
  }

	public FollowAprilTag(CommandSwerveDrivetrain m_sd, Transform3d _target) {
    super(m_sd);
    targetToGoal = _target;
  }


  @Override
  public void execute(){

   /*  
    if (VisionSystem.hasVisionMeasure){
      hasFirstTarget=true;
      hasTarget=true;

      Transform3d temp =AprilTagCam.transformTagToRobot;
      targetTransform= new Transform3d(
          temp.getX(),
          temp.getY(),
          temp.getZ(),
          new Rotation3d(0, 0,temp.getRotation().toRotation2d().getRadians()+Math.PI)
          );
          SmartDashboard.putNumber("TargetTransmfor",temp.getX());
    }
    super.execute();
 */
    }
    
}