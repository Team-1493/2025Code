package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionConstants {
    
    private static double fieldLength=16.54,fieldWidth=8.21;


    
    public static List<AprilTag> AprilTagList = new ArrayList<AprilTag>(Arrays.asList(

    new AprilTag(1, new Pose3d(16.697,	0.655, 1.486, new Rotation3d(0.000, 0.000, 2.199))),
    new AprilTag(2, new Pose3d(16.697,	7.396, 1.486, new Rotation3d(0.000, 0.000, 4.084))),
    new AprilTag(3, new Pose3d(11.561,	8.056, 1.302, new Rotation3d(0.000, 0.000, 4.712))),
    new AprilTag(4, new Pose3d(9.276, 6.138,	1.868, new Rotation3d(0.000,	0.524, 0.000))),
    new AprilTag(5, new Pose3d(9.276, 1.915,	1.868, new Rotation3d(0.000,	0.524, 0.000))),
    new AprilTag(6, new Pose3d(13.474,	3.306, 0.308, new Rotation3d(0.000, 0.524, 5.236))),
    new AprilTag(7, new Pose3d(13.891,	4.026, 0.308, new Rotation3d(0.000, 0.000, 0.000))),
    new AprilTag(8, new Pose3d(13.474,	4.745, 0.308, new Rotation3d(0.000, 0.000, 1.047))),
    new AprilTag(9, new Pose3d(12.643,	4.745, 0.308, new Rotation3d(0.000, 0.000, 2.094))),
    new AprilTag(10, new Pose3d(12.227, 4.026, 0.308, new Rotation3d(0.000, 0.000, 3.142))),
    new AprilTag(11, new Pose3d(12.643, 3.306, 0.308, new Rotation3d(0.000, 0.000, 4.189))),
    new AprilTag(12, new Pose3d(0.851, 0.655, 1.486, new Rotation3d(0.000, 0.000, 0.942))),
    new AprilTag(13, new Pose3d(0.851, 7.396, 1.486, new Rotation3d(0.000, 0.524, 5.341))),
    new AprilTag(14, new Pose3d(8.272,	6.138, 1.868, new Rotation3d(0.000, 0.524, 3.142))),
    new AprilTag(15, new Pose3d(8.272,	1.915, 1.868, new Rotation3d(0.000, 0.000, 3.142))),
    new AprilTag(16, new Pose3d(5.988,	-0.004,	0.308, new Rotation3d(0.000,	0.000, 1.571))),
    new AprilTag(17, new Pose3d(4.074,	3.306,  0.308, new Rotation3d(0.000, 0.000, 4.189))),
    new AprilTag(18, new Pose3d(3.658,	4.026, 0.308, new Rotation3d(0.000, 0.000, 3.142))),
    new AprilTag(19, new Pose3d(4.074,	4.745, 0.308, new Rotation3d(0.000, 0.000, 2.094))),
    new AprilTag(20, new Pose3d(4.905,	4.745, 0.308, new Rotation3d(0.000, 0.000, 1.047))),
    new AprilTag(21, new Pose3d(5.321,	4.026, 0.308, new Rotation3d(0.000, 0.000, 0.000))),
    new AprilTag(22, new Pose3d(4.905,	3.306, 0.308, new Rotation3d(0.000, 0.000, 5.236)))
    ));
    
    static public AprilTagFieldLayout FieldLayout = new AprilTagFieldLayout(AprilTagList, fieldLength, fieldWidth);
}