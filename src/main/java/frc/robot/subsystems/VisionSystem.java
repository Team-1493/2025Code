package frc.robot.subsystems;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Utilities.VisionConstants;

public class VisionSystem extends SubsystemBase {

    CommandSwerveDrivetrain dt;

    AprilTagCam camFL,camFR,camB;

    public static boolean hasSysVisionMeasure=false;
    public static boolean hasReefTarget;
    public static int closestReefID=0;
    public static double closestReefDist=999;
    public static double reefOffsetX=.11;//0.185
    public static double reefOffsetY=.29;//.210
    public static double intakeOffsetX=0;
    public static double intakeOffsetY=.5;
    public static double stdFactor=30;


    Transform3d camFR_RobotToCam =
        new Transform3d(0,-.247,.188,new Rotation3d(0,0,Math.toRadians(33)));
    Transform3d camFL_RobotToCam = 
        new Transform3d(0,.247,.188,new Rotation3d(0,0,Math.toRadians(-30)));
    
    Transform3d camB_RobotToCam = 
        new Transform3d(-.2794,0,1.003,new Rotation3d(0,.2146,Math.PI));

    public static Matrix<N3, N1> kSingleTagStdDevs;
    public static Matrix<N3, N1> kMultiTagStdDevs;

//    private VisionSystemSim visionSim;

    

    public VisionSystem(CommandSwerveDrivetrain m_dt) {

        SmartDashboard.putNumber("std single X", .3);    
        SmartDashboard.putNumber("std single Y", .3);    
        SmartDashboard.putNumber("std single Rot", 9999);    

        SmartDashboard.putNumber("std multi X", .25);    
        SmartDashboard.putNumber("std multi Y", .25);    
        SmartDashboard.putNumber("std multi Rot", 2);
        SmartDashboard.putNumber("std factor", stdFactor);

        SmartDashboard.putNumber("ReefOffsetX", reefOffsetX);
        SmartDashboard.putNumber("ReefOffsetY", reefOffsetY);
        SmartDashboard.putNumber("IntakeOffsetX", intakeOffsetX);
        SmartDashboard.putNumber("IntakeOffsetY", intakeOffsetY);

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

//        camB = new AprilTagCam("OV9281_1", camB_RobotToCam,
//               dt);           
            



    }

        

  @Override
  public void periodic(){ 
            camFL.getEstimatedGlobalPose();
            camFR.getEstimatedGlobalPose();
//            camB.getEstimatedGlobalPose();
//            visionSim.update(dt.getPose());
    }


    public void configure(){
        kSingleTagStdDevs = VecBuilder.fill(
                SmartDashboard.getNumber("std single X", 0), 
                SmartDashboard.getNumber("std single Y", 0),  
                SmartDashboard.getNumber("std single Rot", 0));

        kMultiTagStdDevs = VecBuilder.fill(
                    SmartDashboard.getNumber("std multi X", 0), 
                    SmartDashboard.getNumber("std multi Y", 0),  
                    SmartDashboard.getNumber("std multi Rot", 0));

        stdFactor = SmartDashboard.getNumber("std factor", stdFactor);

        reefOffsetX = SmartDashboard.getNumber("ReefOffsetX", reefOffsetX);
        reefOffsetY = SmartDashboard.getNumber("ReefOffsetY", reefOffsetY);
        intakeOffsetX = SmartDashboard.getNumber("IntakeOffsetX", intakeOffsetX);
        intakeOffsetY = SmartDashboard.getNumber("IntakeOffsetY", intakeOffsetY);                    
                    
    }

}
