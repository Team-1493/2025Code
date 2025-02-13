package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {

    CommandSwerveDrivetrain dt;

    AprilTagCam camFL,camFR,camB;

    public static boolean hasVisionMeasure=false;

    Transform3d camFR_RobotToCam = 
        new Transform3d(.1,.1,.22,new Rotation3d(0,0,0));
    Transform3d camFL_RobotToCam = 
        new Transform3d(-.1,-.1,.22,new Rotation3d(0,0,Math.toRadians(-30)));
    
    Transform3d camB_RobotToCam = 
        new Transform3d(.1,.1,.1,new Rotation3d(0,0,0));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // TODO - pick correct deviations
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(6, 6, 8);//8
    
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.25, 0.25, 1);//.5 .5 1


    static public double visionStdDevFactor = 0.5;


    public VisionSystem(CommandSwerveDrivetrain m_dt) {
        hasVisionMeasure=false;
        dt=m_dt;
        camFL = new AprilTagCam("Spinel_1", camFL_RobotToCam,kSingleTagStdDevs,kMultiTagStdDevs,dt);
//        camFR = new AprilTagCam("Spinel_2", camFR_RobotToCam,kSingleTagStdDevs,kMultiTagStdDevs);
//        camB = new AprilTagCam("OV9281_1", camB_RobotToCam,kSingleTagStdDevs,kMultiTagStdDevs);

    }

        

  @Override
  public void periodic() {
       if (dt.camState!=2) {
            addVisionMeas(camFL);
            camFL.updateVisionSim(dt.getPose());

       }

    //   if (dt.camState!=1) addVisionMeas(camFL);
    }

    private void addVisionMeas(AprilTagCam cam){
        var visionEst = cam.getEstimatedGlobalPose();
    }



}
