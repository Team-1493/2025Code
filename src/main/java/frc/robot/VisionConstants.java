package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionConstants {
    

    static public Transform3d ROBOT_TO_CAMERA_B = new Transform3d(
        -0.178, 
        -0.127, 
        0.993,//0.508, 
        new Rotation3d(
            0.0, 
            -0.23,//0.5236,
            Math.PI)
    );

    static public Transform3d ROBOT_TO_CAMERA_F = new Transform3d(
        0.3175, 0, 0.65, 
        new Rotation3d(0.0,0.138, 0)
    );  

        static public Transform3d NOTE_ROBOT_TO_CAMERA = new Transform3d(
        0.3175,.1, 0.65, 
        new Rotation3d(0.0, 0.0, 0.0)
    );

    static public Pose2d AmpActionPose = new Pose2d(new Translation2d(1.842, 7.7), new Rotation2d(-Math.PI/2));
    static public Pose2d SpeakerActionPose = new Pose2d(new Translation2d(1.02, 5.5), new Rotation2d(0));
    static public Pose2d SourceActionPose = new Pose2d(new Translation2d(16.38, 1), new Rotation2d(Math.toRadians(117)));
    static public Pose2d PassActionPose = new Pose2d(new Translation2d(15., 1.1), new Rotation2d(Math.toRadians(-40)));    
    static public Pose2d SourceGroundActionPose = new Pose2d(new Translation2d(15.5, 1.5), new Rotation2d(Math.toRadians(-63))); //was 16, 1
    static public double SpeakerRadius = 2.64;

    static public double visionStdDevFactor = 0.5;

    private static double fieldLength=16.54,fieldWidth=8.21;

    private static List<AprilTag> AprilTagList = new ArrayList<AprilTag>(Arrays.asList(
        new AprilTag(1, new Pose3d(15.08, 0.246, 1.356, new Rotation3d(0, 0, 2.094))),
        new AprilTag(2, new Pose3d(16.185, 0.884, 1.356, new Rotation3d(0, 0, 2.094))),
        new AprilTag(3, new Pose3d(16.579, 4.983, 1.451, new Rotation3d(0, 0, 3.142))),

        new AprilTag(4, new Pose3d(16.579, 5.548, 1.451, new Rotation3d(0, 0, 3.142))),
        new AprilTag(5, new Pose3d(14.701, 8.204, 1.356, new Rotation3d(0, 0, 4.712))),
        new AprilTag(6, new Pose3d(1.842, 8.204, 1.356, new Rotation3d(0, 0, 4.712))),
        
        new AprilTag(7, new Pose3d(-0.038, 5.548, 1.451, new Rotation3d(0, 0, 0))),
        
        new AprilTag(8, new Pose3d(-0.038, 4.983, 1.451, new Rotation3d(0, 0, 0))),
        
        new AprilTag(9, new Pose3d(0.356, 0.884, 1.356, new Rotation3d(0, 0, 1.047))),

        new AprilTag(10, new Pose3d(1.462, 0.246, 1.356, new Rotation3d(0, 0, 1.047))),
        new AprilTag(11, new Pose3d(11.905, 3.713, 1.321, new Rotation3d(0, 0, 5.236))),
        new AprilTag(12, new Pose3d(11.905, 4.498, 1.321, new Rotation3d(0, 0, 1.047))),
        new AprilTag(13, new Pose3d(11.22, 4.105, 1.321, new Rotation3d(0, 0, 3.142))),
        new AprilTag(14, new Pose3d(5.321, 4.105, 1.321, new Rotation3d(0, 0, 0))),
        new AprilTag(15, new Pose3d(4.641, 4.498, 1.321, new Rotation3d(0, 0, 2.094))),
        new AprilTag(16, new Pose3d(4.641, 3.713, 1.321, new Rotation3d(0, 0, 4.189)))
    ));
    
    static public AprilTagFieldLayout FieldLayout = new AprilTagFieldLayout(AprilTagList, fieldLength, fieldWidth);
}
