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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionConstants {
    boolean blue=true;
    double fieldLength=16.54,fieldWidth=8.21;
    public List<AprilTag> aprilTagList =  new ArrayList<AprilTag>(Arrays.asList(

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



    public AprilTagFieldLayout FieldLayout;
  

    public VisionConstants(){

        getAlliance();
    
    if(!blue || blue){   
        System.out.println("*******************   RED  ******");
        double pi = Math.PI;
    aprilTagList = new ArrayList<AprilTag>(Arrays.asList(

    new AprilTag(1, new Pose3d(17.55-16.697,	8.051-0.655, 1.486, new Rotation3d(0.000, 0.000, 2.199+pi))),
    new AprilTag(2, new Pose3d(17.55-16.697,	8.051-7.396, 1.486, new Rotation3d(0.000, 0.000, 4.084-pi))),
    new AprilTag(3, new Pose3d(17.55-11.561,	8.051-8.056, 1.302, new Rotation3d(0.000, 0.000, 4.712-pi))),
    new AprilTag(4, new Pose3d(17.55-9.276, 8.051-6.138,	1.868, new Rotation3d(0.000,	0.524, 0.000+pi))),
    new AprilTag(5, new Pose3d(17.55-9.276, 8.051-1.915,	1.868, new Rotation3d(0.000,	0.524, 0.000+pi))),
    new AprilTag(6, new Pose3d(17.55-13.474,	8.051-3.306, 0.308, new Rotation3d(0.000, 0.524, 5.236-pi))),
    new AprilTag(7, new Pose3d(3.658,	8.051-4.026, 0.308, new Rotation3d(0.000, 0, 0+pi))),
    new AprilTag(8, new Pose3d(17.55-13.474,	8.051-4.745, 0.308, new Rotation3d(0.000, 0.000, 1.047+pi))),
    new AprilTag(9, new Pose3d(17.55-12.643,	8.051-4.745, 0.308, new Rotation3d(0.000, 0.000, 2.094+pi))),
    new AprilTag(10, new Pose3d(17.55-12.227, 8.051-4.026, 0.308, new Rotation3d(0.000, 0.000, 0))),
    new AprilTag(11, new Pose3d(17.55-12.643, 8.051-3.306, 0.308, new Rotation3d(0.000, 0.000, 4.189-pi))),
    new AprilTag(12, new Pose3d(17.55-0.851, 8.051-0.655, 1.486, new Rotation3d(0.000, 0.000, 0.942+pi))),
    new AprilTag(13, new Pose3d(17.55-0.851, 8.051-7.396, 1.486, new Rotation3d(0.000, 0.524, 5.341+pi))),
    new AprilTag(14, new Pose3d(17.55-8.272,	8.051-6.138, 1.868, new Rotation3d(0.000, 0.524, 0+pi))),
    new AprilTag(15, new Pose3d(17.55-8.272,	8.051-1.915, 1.868, new Rotation3d(0.000, 0.000, 0+pi))),
    new AprilTag(16, new Pose3d(17.55-5.988,	8.051+0.004,	0.308, new Rotation3d(0.000,	0.000, 1.571+pi))),
    new AprilTag(17, new Pose3d(17.55-4.074,	8.051-3.306,  0.308, new Rotation3d(0.000, 0.000, 4.189-pi))),
    new AprilTag(18, new Pose3d(3.658,	8.051-4.026, 0.308, new Rotation3d(0.000, 0.000, 0+pi))),
    new AprilTag(19, new Pose3d(17.55-4.074,	8.051-4.745, 0.308, new Rotation3d(0.000, 0.000, 2.094+pi))),
    new AprilTag(20, new Pose3d(17.55-4.905,	8.051-4.745, 0.308, new Rotation3d(0.000, 0.000, 1.047+pi))),
    new AprilTag(21, new Pose3d(17.55-5.321,	8.051-4.026, 0.308, new Rotation3d(0.000, 0.000, 0.000+pi))),
    new AprilTag(22, new Pose3d(17.55-4.905,	8.051-3.306, 0.308, new Rotation3d(0.000, 0.000, 5.236-pi)))
    ));
    }
  


   FieldLayout  = new AprilTagFieldLayout(aprilTagList, fieldLength, fieldWidth);
   System.out.println("********************  "+aprilTagList.get(21).pose.toPose2d().getX());
}



public void getAlliance(){
  
   
    DriverStation.getAlliance().ifPresent(allianceColor -> {
            if (allianceColor == Alliance.Red) blue=false;
    });

    SmartDashboard.putBoolean("Alliance", blue);
}

}