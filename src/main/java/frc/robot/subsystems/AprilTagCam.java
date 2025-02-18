/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.subsystems;
 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.VisionConstants;

import java.util.List;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

 
 public class AprilTagCam {
    public final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private CommandSwerveDrivetrain dt;
     public double closestTargetDist,closestTargetX,closestTargetY,closestTargetRot;
    public int closestTargetID=0;
    private Matrix<N3, N1> curStdDevs;
    private String  labelX,labelY,labelRot,labelHas,
                    labelDevX,labelDevY,labelDevRot, labelClosestID, 
                    labelClosestDist; 
    boolean hasVisionMeasure=false;
    public int numTargets;
 

     // *** For Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;
    // ***
     
     public AprilTagCam(
                String kCameraName, 
                Transform3d kRobotToCam,
                CommandSwerveDrivetrain m_dt,
                VisionSystemSim m_visionSystemSim) {

        visionSim = m_visionSystemSim;

        dt=m_dt;

        labelX=kCameraName+" X";
        labelY=kCameraName+" Y";
        labelRot=kCameraName+" Rot";
        labelDevX=kCameraName+" DevX";
        labelDevY=kCameraName+" DevY";
        labelDevRot=kCameraName+" DevRot";
        labelHas=kCameraName+" Has Target";
        labelClosestID=kCameraName+" ClosestID";
        labelClosestDist=kCameraName+" ClosestDist";
        

        camera = new PhotonCamera(kCameraName);
        
 
        photonEstimator =
                 new PhotonPoseEstimator(VisionConstants.FieldLayout, 
                 PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
                 photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
         // ----- Simulation
         if(Robot.isSimulation()){
             // Create simulated camera properties. These can be set to mimic your actual camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
             // targets.
             cameraSim = new PhotonCameraSim(camera, cameraProp);
             cameraSim.setMaxSightRange(4);
             // Add the simulated camera to view the targets on this simulated field.
             visionSim.addCamera(cameraSim, kRobotToCam);
            
             cameraSim.enableDrawWireframe(false);
         }
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
//         System.out.println("size "+camera.getAllUnreadResults().size());
        var pipelineResults = camera.getAllUnreadResults();
        if(pipelineResults.size()>0){
            closestTargetDist=9999;
            closestTargetID=0;
             hasVisionMeasure=false;    
        }
         for (var change : pipelineResults) {
            visionEst = photonEstimator.update(change);
//            System.out.println("*****   "+visionEst.isPresent());
            if (visionEst.isPresent()) 
                {
                if (change.hasTargets()) hasVisionMeasure=true;

                updateEstimationStdDevs(visionEst, change.getTargets());
                dt.addVisionMeasurement(
                    visionEst.get().estimatedPose.toPose2d(),
                    Utils.fpgaToCurrentTime(visionEst.get().timestampSeconds), curStdDevs);
                printResults(visionEst);
                }


             if (Robot.isSimulation()) {
                 visionEst.ifPresentOrElse(
                         est ->
                                 getSimDebugField()
                                         .getObject("VisionEstimation")
                                         .setPose(est.estimatedPose.toPose2d()),
                         () -> {
                             getSimDebugField().getObject("VisionEstimation").setPoses();
                         });
             }
         }
         
 
        SmartDashboard.putBoolean("VisionEstPresent", visionEst.isPresent());
        
        return visionEst;

     }
 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     private void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = VisionSystem.kSingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = VisionSystem.kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
             double dist;
             
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 dist=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

                
//                System.out.println("ID = "+tgt.getFiducialId()+"     amb = "+tgt.getPoseAmbiguity());
                avgDist += dist;

                 if (dist<closestTargetDist) {
                    closestTargetDist=dist;
                    closestTargetID=tgt.getFiducialId();
             }                                 
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs = VisionSystem.kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = VisionSystem.kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 10));//30
                 curStdDevs = estStdDevs;
             }
         }
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }
 
     // ----- Simulation
 

     public void simulationPeriodic(Pose2d robotSimPose) {

     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation()) return null;
         return visionSim.getDebugField();
     }


    private void printResults(Optional<EstimatedRobotPose> visionEst){
        
        if(hasVisionMeasure){
        SmartDashboard.putBoolean(labelHas, hasVisionMeasure);

        Pose2d estpose2d = visionEst.get().estimatedPose.toPose2d();
        SmartDashboard.putNumber(
            labelX,
            estpose2d.getX());

        SmartDashboard.putNumber(
            labelY,
            estpose2d.getY());
        
        SmartDashboard.putNumber(
            labelRot,
            estpose2d.getRotation().getDegrees());  

        SmartDashboard.putNumber(
            labelDevX,
            curStdDevs.get(0,0));

        SmartDashboard.putNumber(
            labelDevY,
            curStdDevs.get(1,0));

        SmartDashboard.putNumber(
            labelDevRot,
            curStdDevs.get(2,0));
                
        SmartDashboard.putNumber(
            labelClosestID,
            closestTargetID); 
                
        SmartDashboard.putNumber(
            labelClosestDist,
            closestTargetDist);                         
        }

        else SmartDashboard.putBoolean(labelHas, hasVisionMeasure);

     }

    

 }