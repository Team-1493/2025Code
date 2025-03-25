package frc.robot.subsystems;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSystem extends SubsystemBase {

    CommandSwerveDrivetrain dt;

    AprilTagCam camFL,camFR,camB;

    public static boolean hasSysVisionMeasure=false;
    public static boolean hasReefTarget;
    public static int closestReefID=0;
    public static double closestReefDist=999;
    public static double reefOffsetXLeft=.0846;//0.185
    public static double reefOffsetXRight=.13;//0.11
    public static double reefOffsetY=.21;//  *****  .29 for tech park  *****
    public static double intakeOffsetX=0;
    public static double intakeOffsetY=0;
    public static double stdFactor=30;
    public static boolean towardsIntake=false;    



    Transform3d camFR_RobotToCam =
        new Transform3d(0,-.247,.188,new Rotation3d(0,0,Math.toRadians(33)));
    Transform3d camFL_RobotToCam = 
        new Transform3d(0,.247,.188,new Rotation3d(0,0,Math.toRadians(-30)));
    
    Transform3d camB_RobotToCam = 
        new Transform3d(-.1016,0,1.003,new Rotation3d(0,Math.toRadians(-28.8),Math.PI));
//.2146
    public static Matrix<N3, N1> kSingleTagStdDevs;
    public static Matrix<N3, N1> kMultiTagStdDevs;

//    private VisionSystemSim visionSim;

    

    public VisionSystem(CommandSwerveDrivetrain m_dt) {
        configure();


        if (Robot.isSimulation()) {
            
             // Create the vision system simulation which handles cameras and targets on the //field.
      //       visionSim = new VisionSystemSim("visionSim");
             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      //       visionSim.addAprilTags(VisionConstants.FieldLayout);
        }

        dt=m_dt;

        camFL = new AprilTagCam("Spinel_L", camFL_RobotToCam,
            dt);
        
        camFR = new AprilTagCam("Spinel_R", camFR_RobotToCam,
            dt);

        camB = new AprilTagCam("Spinel_B", camB_RobotToCam,
               dt);           
            



    }

        

  @Override
  public void periodic(){
    
        camFL.getEstimatedGlobalPose();
        camFR.getEstimatedGlobalPose();
     
    if (towardsIntake)  camB.getEstimatedGlobalPose();
//            visionSim.update(dt.getPose());
    }


    public void configure(){
        kSingleTagStdDevs = VecBuilder.fill(0.3,0.3,9999);

        kMultiTagStdDevs = VecBuilder.fill(0.25,0.25,2);

        stdFactor = SmartDashboard.getNumber("std factor", stdFactor);                    
    }

}
