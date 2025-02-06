package frc.robot.subsystems;

import java.util.Optional;

import javax.swing.text.rtf.RTFEditorKit;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;

public class AprilTagCam extends SubsystemBase{
    public final PhotonCamera cameraB = new PhotonCamera("OV9281_2");
    public final PhotonCamera cameraF = new PhotonCamera("OV9281_1");
    public final AprilTagFieldLayout fieldLayout;

    public final PhotonPoseEstimator poseEstimatorF,poseEstimatorB;
    static PhotonPipelineResult latestResultF,latestResultB;
    public static Boolean hasVisionMeasurementF = false,hasVisionMeasurementB=false;
    public static Boolean hasVisionMeasurement=false;
    public static Pose2d visionPoseF = new Pose2d(), visionPoseB = new Pose2d();
    public static Pose2d visionPose = new Pose2d();
    public static Transform3d transformTagToRobotF = new Transform3d();
    public static Transform3d transformTagToRobotB = new Transform3d();   
    public static Transform3d transformTagToRobot = new Transform3d();        
    static double visionTimeStampF = 0, visionTimeStampB=0, visionTimeStamp;
    static double visionLeastDistance = 1;
    static Vector<N3> visionStdDevs;
    double distF,distB,leastDist;
    public static boolean FRONT=false,BACK=false;


    Optional <EstimatedRobotPose> estPoseF = null;
    Optional <EstimatedRobotPose> estPoseB = null;

    // *** This is used for camera siumulation
    VisionSystemSim visionSim = new VisionSystemSim("visionSim");
    TargetModel targetModel = TargetModel.kAprilTag36h11;
    SimCameraProperties cameraProp = new SimCameraProperties();
    PhotonCameraSim cameraSim;
    
    //***


    public AprilTagCam(){

        SmartDashboard.putNumber("Vision Pose", 0);
        SmartDashboard.putNumber("Vision Pose", 0);
        SmartDashboard.putNumber("Vision Pose", 0);  
        SmartDashboard.putBoolean("Vision Has MeasurementF", hasVisionMeasurementF);
        SmartDashboard.putBoolean("Vision Has MeasurementB", hasVisionMeasurementB);        

        fieldLayout = VisionConstants.FieldLayout;


        poseEstimatorF = new PhotonPoseEstimator(fieldLayout,
        PoseStrategy.LOWEST_AMBIGUITY,
        VisionConstants.ROBOT_TO_CAMERA_F);

        poseEstimatorB = new PhotonPoseEstimator(fieldLayout,
        PoseStrategy.LOWEST_AMBIGUITY,
        VisionConstants.ROBOT_TO_CAMERA_B);        


        // ***** For camera simulation
        visionSim.addAprilTags(fieldLayout);
        cameraProp.setCalibration(640,
                                 480,
                                Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(cameraF, cameraProp);

        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA_F);
        visionSim.getDebugField();

        // ***

    }

    public void periodic(){

        hasVisionMeasurement=false;
        latestResultF = cameraF.getLatestResult();
        estPoseF = poseEstimatorF.update(latestResultF);
        hasVisionMeasurementF = estPoseF.isPresent();
        if (hasVisionMeasurementF){
            transformTagToRobotF=latestResultF.getBestTarget().getBestCameraToTarget();
            //Turns the estimated pose from a pose3d to pose2d
            visionPoseF = estPoseF.get().estimatedPose.toPose2d();
            //Gets the time stamp of the estimated pose
            visionTimeStampF = estPoseF.get().timestampSeconds;   
        }  
        
        latestResultB = cameraB.getLatestResult();
        estPoseB = poseEstimatorB.update(latestResultB);
        hasVisionMeasurementB = estPoseB.isPresent();
        if (hasVisionMeasurementB){
           transformTagToRobotB=latestResultB.getBestTarget().getBestCameraToTarget();
            //Turns the estimated pose from a pose3d to pose2d
            visionPoseB = estPoseB.get().estimatedPose.toPose2d();
            //Gets the time stamp of the estimated pose
            visionTimeStampB = estPoseB.get().timestampSeconds;
        }           

        // Determine which camera has closest target, use that one for vision measurement
        // and calculate SD (proportional to distance)
        FRONT=false;BACK=false;
        SmartDashboard.putBoolean("Vision Has MeasurementF", hasVisionMeasurementF);
        SmartDashboard.putBoolean("Vision Has MeasurementB", hasVisionMeasurementB);
        if (hasVisionMeasurementB || hasVisionMeasurementF) {
            distF=999;
            distB=999;
            if(hasVisionMeasurementF) distF = latestResultF.getBestTarget().
                getBestCameraToTarget().getTranslation().getNorm();
            if(hasVisionMeasurementB) distB = latestResultB.getBestTarget().
                getBestCameraToTarget().getTranslation().getNorm();
            leastDist=Math.min(distF, distB);
            SmartDashboard.putNumber("leastDist", leastDist);

// if closest target is more than 2 meters aways don't use vision measurement
// could also change to an ambiguity threshold            
            if (leastDist<6){
            
                visionStdDevs = VecBuilder.fill(
                    visionLeastDistance * VisionConstants.visionStdDevFactor,
                    visionLeastDistance * VisionConstants.visionStdDevFactor,
                    99999
                );

                if (distB<distF){
                    BACK=true;
                    visionPose=visionPoseB;
                    visionTimeStamp=visionTimeStampB;
                    transformTagToRobot=transformTagToRobotF;
                }
                else{
                    FRONT=true;
                    visionPose=visionPoseF;
                    visionTimeStamp=visionTimeStampF;
                    transformTagToRobot=transformTagToRobotF;
                }
                hasVisionMeasurement=true;
                SmartDashboard.putBoolean("Vision B", BACK);
                SmartDashboard.putBoolean("Vision F", FRONT);
                SmartDashboard.putNumber("Vision PoseX", visionPose.getX());
                SmartDashboard.putNumber("Vision PoseY", visionPose.getY());
                SmartDashboard.putNumber("Vision PoseZ", visionPose.getRotation().getDegrees());  
                SmartDashboard.putNumber("TransformTagToRobot X",transformTagToRobot.getX());
            }
            else hasVisionMeasurement=false;

        }

    }

    public void simulatedPeriodic(){

        visionSim.update(CommandSwerveDrivetrain.robotpose);
    }




}
