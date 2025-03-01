package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {


//  **************  DRIVE CONSTANTS   *********************************
    static double drive_kP=3;
    static double drive_kI=0;
    static double drive_kD=0;
    static double drive_kS=0;
    static double drive_kV=0;
    static double drive_kA=0;

//  **************  CLIMBER CONSTANTS   *********************************    
    static public double climber_BOTPOS=1;
    static public double climber_TOPPOS=210;
    static public double climber_CLIMBPOS=5;

    static public double climber_kP=20;
    static public double climber_kI = 20;
    static public double climber_kD = .5;
    static public double climber_kG = 0;    

    static public double climber_RevSoftLim = 0;
    static public double climber_ForSoftLim = 210;    

    static public double climber_PeakTC = 40;



    static public double climberPeakVolt = 1;    

    //  **************  ARM CONSTANTS   *********************************
    static public double ArmGroundPos = 0.09;
    static public double ArmMidPos = 0.09;
    static  public double ArmSourcePos = 0.275;
    static public double ArmAmpPos = 0.275;
    static public double ArmSpeakerClosePos = 0.275;
    static public double ArmSpeakerFarPos = 0.09;


    static double arm_MagneticOffset = -0.15;
    static double arm_ForwardSoftLimitThreshold = 0.275;
    static double arm_ReverseSoftLimitThreshold = 0.10;
 

// From Chief Delphi example
//    kS = 1.6221;
//    kG = 2.3923;
//    kV = 0  (1.6429);
//    kA = 2.7332;
//    kP = 69.657;
//    kI = 0;
//    kD = 26.77;

    static double arm_kA=0;
    static double arm_kV = 0;
    static double arm_kP = 200;
    static double arm_kI = 200; 
    static double arm_kD = 50;
    static double arm_kG = 15;
    static double arm_MMCruiseVelocity = 1; 
    static double arm_MMAcceleration = 2; // 
    static double arm_MMJerk = 0; // no limit when set to zero
    static double arm_StatorCurrentLimit=200;    
    static double arm_PeakTC = 400;
    static double arm_PeakVolt=12;
    
//  **************  TRADITIONAL SHOOTER CONSTANTS  ******************
    public static double shooterAmpR = 700;
    public static double shooterAmpL = 700;
    public static double shooterSpeakerCloseR = 3000;//3250;//3500;
    public static double shooterSpeakerCloseL = 2500;//2750;//3000;
    public static double shooterSpeakerCloseAutoR = 3000;
    public static double shooterSpeakerCloseAutoL = 2500;
    public static double shooterSpeakerFarR = 2000;
    public static double shooterSpeakerFarL = 3200;
    public static double shooterMidR = 1900;
    public static double shooterMidL = 3000;
    public static double shooterPass1R = 2500;
    public static double shooterPass1L = 3200;
    public static double shooterPass2R = 2000;
    public static double shooterPass2L = 3200;
    static double shooterOLRampPeriod =0.03;
    static double shooterCLRampPeriod =0;
    static double shooterStatorLim =250;
    static double shooterSupplyLim =40;
    static double shooterSupplyThresh =40;
    static double shooterSupplyTime =0.05;
    static double shooterPeakTC=600;
    static double shooter_kP=11;
    static double shooter_kV=0;
    static double shooter_kA=0;
    static double shooter_kI=4;
    static double ampReboundForVolt=-2;
    static double ampReboundRevVolt=0.65;

    
//  **************  AUTO CONSTANTS  ******************
    static double autoBuilder_Trans_kP=10;
    static double autoBuilder_Rot_kP=10;
    static double autoBuilder_Trans_kD=0;
    static double autoBuilder_Rot_kD=0;
    static double constraintVel=1;
    static double constraintAcc=2;
    static double constraintRotVel=360;
    static double constraintRotAcc=720;


    
//  **************  FOLLOW POSE CONSTANTS  ******************
    public static double followPose_Trans_kP=4;
    public static double followPose_Trans_kD=.1;
    public static double followPose_Rot_kP=4;
    public static double followPose_Rot_kD=.5;
    public static double followPose_Trans_maxV=2;
    public static double followPose_Trans_maxA=8;
    public static double followPose_Rot_maxV=1;
    public static double followPose_Rot_maxA=2;

//  **** Camera Constants
    static public double aprilTagSF_front = 0.97;
    static public double aprilTagSF_back = 0.97;


    public Constants(){

    }


    static public void printConstants(){
        // ****** PRINT SHOOTER CONSTANTS
        SmartDashboard.putNumber("Shooter Amp R",shooterAmpR);
        SmartDashboard.putNumber("Shooter Amp L",shooterAmpL);
        SmartDashboard.putNumber("Shooter Speaker Close R",shooterSpeakerCloseR);
        SmartDashboard.putNumber("Shooter Speaker Close L",shooterSpeakerCloseL);
        SmartDashboard.putNumber("Shooter Speaker Far R",shooterSpeakerFarR);
        SmartDashboard.putNumber("Shooter Speaker Far L",shooterSpeakerFarL);
        SmartDashboard.putNumber("Shooter Mid R",shooterMidR);
        SmartDashboard.putNumber("Shooter Mid L",shooterMidL);
        SmartDashboard.putNumber("Shooter Pass1 R",shooterPass1R);
        SmartDashboard.putNumber("Shooter Pass1 L",shooterPass1L);        
        SmartDashboard.putNumber("Shooter Pass2 R",shooterPass2R);
        SmartDashboard.putNumber("Shooter Pass2 L",shooterPass2L);                

        SmartDashboard.putNumber("Shooter OL Ramp", shooterOLRampPeriod);
        SmartDashboard.putNumber("Shooter CL Ramp", shooterCLRampPeriod);
        SmartDashboard.putNumber("Shooter Stator Lim", shooterStatorLim);
        SmartDashboard.putNumber("Shooter Supply Lim", shooterSupplyLim);
        SmartDashboard.putNumber("Shooter Supply Thresh", shooterSupplyThresh);
        SmartDashboard.putNumber("Shooter Supply Time", shooterSupplyTime);
        SmartDashboard.putNumber("Shooter PeakTC", shooterPeakTC);
        SmartDashboard.putNumber("Shooter kP", shooter_kP);
        SmartDashboard.putNumber("Shooter kI", shooter_kI);
        SmartDashboard.putNumber("Shooter kV", shooter_kV);
        SmartDashboard.putNumber("Shooter kA", shooter_kA);
        SmartDashboard.putNumber("Shooter Amp Rebound For", ampReboundForVolt);
        SmartDashboard.putNumber("Shooter Amp Rebound Rev", ampReboundRevVolt);


        // ******* PRINT CLIMBER CONSTANTS
        SmartDashboard.putNumber("Climber Bot Pos",climber_BOTPOS);
        SmartDashboard.putNumber("Climber Top Pos",climber_TOPPOS);
        SmartDashboard.putNumber("Climber Climb Pos",climber_CLIMBPOS);        
        SmartDashboard.putNumber("Climber kP",climber_kP);
        SmartDashboard.putNumber("Climber kI",climber_kI); 
        SmartDashboard.putNumber("Climber kD",climber_kD);
        SmartDashboard.putNumber("Climber kG",climber_kG);        
        SmartDashboard.putNumber("Climber Peak Volt",climberPeakVolt);    
        SmartDashboard.putNumber("Climber RevSoftLim",climber_RevSoftLim);
        SmartDashboard.putNumber("Climber ForSoftLim",climber_ForSoftLim);
        SmartDashboard.putNumber("Climber PeakTC",climber_PeakTC);

        


        // ****** PRINT ARM CONSTANTS 
        SmartDashboard.putNumber("Arm GroundPos", ArmGroundPos);
        SmartDashboard.putNumber("Arm MidPos", ArmMidPos);
        SmartDashboard.putNumber("Arm SourcePos", ArmSourcePos);
        SmartDashboard.putNumber("Arm SpeakerClosePos", ArmSpeakerClosePos);
        SmartDashboard.putNumber("Arm SpeakerFarPos", ArmSpeakerFarPos);
        SmartDashboard.putNumber("Arm AmpPos", ArmAmpPos);
        SmartDashboard.putNumber("Arm Mag Offset", arm_MagneticOffset);
        SmartDashboard.putNumber("Arm ForSLS", arm_ForwardSoftLimitThreshold);
        SmartDashboard.putNumber("Arm RevSLS", arm_ReverseSoftLimitThreshold);
        SmartDashboard.putNumber("Arm kA", arm_kA);
        SmartDashboard.putNumber("Arm kP", arm_kP);
        SmartDashboard.putNumber("Arm kI", arm_kI);
        SmartDashboard.putNumber("Arm kD", arm_kD);
        SmartDashboard.putNumber("Arm kG", arm_kG);

        SmartDashboard.putNumber("Arm MM vel", arm_MMCruiseVelocity);
        SmartDashboard.putNumber("Arm MM acc", arm_MMAcceleration);
        SmartDashboard.putNumber("Arm StatorCurLim", arm_StatorCurrentLimit);
        SmartDashboard.putNumber("Arm Peak TC",arm_PeakTC);
        SmartDashboard.putNumber("Arm Peak Volt", arm_PeakVolt);

        // ******* PRINT AUTO CONSTANTS
        SmartDashboard.putNumber("Auto AutoBuilder Trans kP", autoBuilder_Trans_kP);
        SmartDashboard.putNumber("Auto AutoBuilder Rot kP", autoBuilder_Rot_kP);
        SmartDashboard.putNumber("Auto AutoBuilder Trans kD", autoBuilder_Trans_kD);
        SmartDashboard.putNumber("Auto AutoBuilder Rot kD", autoBuilder_Rot_kD);
        SmartDashboard.putNumber("Auto Constraint Vel", constraintVel);
        SmartDashboard.putNumber("Auto Constraint Acc", constraintAcc);
        SmartDashboard.putNumber("Auto Constraint Rot Vel", constraintRotVel);
        SmartDashboard.putNumber("Auto Constraint Rot Acc", constraintRotAcc);

        // ******* PRINT FOLLOW POSE CONSTANTS
        SmartDashboard.putNumber("FollowPose Trans kP",followPose_Trans_kP);
        SmartDashboard.putNumber("FollowPose Trans kD",followPose_Trans_kD);
        SmartDashboard.putNumber("FollowPose Rot kP",followPose_Rot_kP);
        SmartDashboard.putNumber("FollowPose Rot kD",followPose_Rot_kD);
        SmartDashboard.putNumber("FollowPose Trans MaxV", followPose_Trans_maxV);
        SmartDashboard.putNumber("FollowPose Trans MaxA", followPose_Trans_maxA);
        SmartDashboard.putNumber("FollowPose Rot MaxV", followPose_Rot_maxV);
        SmartDashboard.putNumber("FollowPose Rot MaxA", followPose_Rot_maxA);


        // ******* PRINT DRIVE CONSTANTS
        SmartDashboard.putNumber("Drive kP", drive_kP);
        SmartDashboard.putNumber("Drive kI", drive_kI);
        SmartDashboard.putNumber("Drive kD", drive_kD);
        SmartDashboard.putNumber("Drive kS", drive_kS);
        SmartDashboard.putNumber("Drive kV", drive_kV);
        SmartDashboard.putNumber("Drive kA", drive_kA);        

        // ********** PRINT CAMERA CONSTANTS
        SmartDashboard.putNumber("Vision AT_SF_front", aprilTagSF_front);
        SmartDashboard.putNumber("Vision AT_SF_back", aprilTagSF_back);        
            

    }


    static public void updateConstants(){

        // ******* UPDATE CLIMBER CONSTANTS
        climber_BOTPOS = SmartDashboard.getNumber("Climber Bot Pos",climber_BOTPOS);
        climber_TOPPOS = SmartDashboard.getNumber("Climber Top Pos",climber_TOPPOS);
        climber_CLIMBPOS = SmartDashboard.getNumber("Climber Climb Pos",climber_CLIMBPOS);        
        climber_kP = SmartDashboard.getNumber("Climber kP",climber_kP);
        climber_kI = SmartDashboard.getNumber("Climber kI",climber_kI); 
        climber_kD = SmartDashboard.getNumber("Climber kD",climber_kD);
        climber_kG = SmartDashboard.getNumber("Climber kG",climber_kG);        
        climberPeakVolt = SmartDashboard.getNumber("Climber Peak Volt",climberPeakVolt);               
        climber_RevSoftLim = SmartDashboard.getNumber("Climber RevSoftLim",climber_RevSoftLim);
        climber_ForSoftLim = SmartDashboard.getNumber("Climber ForSoftLim",climber_ForSoftLim);
        climber_PeakTC = SmartDashboard.getNumber("Climber PeakTC",climber_PeakTC);

        // ******* UPDATE SHOOTER CONSTANTS
        shooterAmpR = SmartDashboard.getNumber("Shooter Amp R",shooterAmpR);
        shooterAmpL = SmartDashboard.getNumber("Shooter Amp L",shooterAmpL);
        shooterSpeakerCloseR = SmartDashboard.getNumber("Shooter Speaker Close R",shooterSpeakerCloseR);
        shooterSpeakerCloseL = SmartDashboard.getNumber("Shooter Speaker Close L",shooterSpeakerCloseL);
        shooterSpeakerFarR = SmartDashboard.getNumber("Shooter Speaker Far R",shooterSpeakerFarR);
        shooterSpeakerFarL = SmartDashboard.getNumber("Shooter Speaker Far L",shooterSpeakerFarL);
        shooterMidR = SmartDashboard.getNumber("Shooter Mid R",shooterMidR);
        shooterMidL = SmartDashboard.getNumber("Shooter Mid L",shooterMidL);
        shooterPass1R = SmartDashboard.getNumber("Shooter Pass1 R",shooterPass1R);
        shooterPass1L = SmartDashboard.getNumber("Shooter Pass1 L",shooterPass1L);        
        shooterPass2R = SmartDashboard.getNumber("Shooter Pass2 R",shooterPass2R);
        shooterPass2L = SmartDashboard.getNumber("Shooter Pass2 L",shooterPass2L);                        

        shooterOLRampPeriod=SmartDashboard.getNumber("Shooter OL Ramp", shooterOLRampPeriod);
        shooterCLRampPeriod=SmartDashboard.getNumber("Shooter CL Ramp", shooterCLRampPeriod);
        shooterStatorLim=SmartDashboard.getNumber("Shooter Stator Lim", shooterStatorLim);
        shooterSupplyLim=SmartDashboard.getNumber("Shooter Supply Lim", shooterSupplyLim);
        shooterSupplyThresh=SmartDashboard.getNumber("Shooter Supply Thresh", shooterSupplyThresh);
        shooterSupplyTime=SmartDashboard.getNumber("Shooter Supply Time", shooterSupplyTime);
        shooterPeakTC = SmartDashboard.getNumber("Shooter PeakTC", shooterPeakTC);        
        shooter_kP=SmartDashboard.getNumber("Shooter kP", shooter_kP);
        shooter_kI=SmartDashboard.getNumber("Shooter kI", shooter_kI);
        shooter_kV=SmartDashboard.getNumber("Shooter kV", shooter_kV);
        shooter_kA=SmartDashboard.getNumber("Shooter kA", shooter_kA);
        ampReboundForVolt = SmartDashboard.getNumber("Shooter Amp Rebound For", ampReboundForVolt);
        ampReboundRevVolt = SmartDashboard.getNumber("Shooter Amp Rebound Rev", ampReboundRevVolt);


        // ******* UPDATE ARM CONSTANTS
        ArmGroundPos = SmartDashboard.getNumber("Arm GroundPos", ArmGroundPos);
        ArmMidPos = SmartDashboard.getNumber("Arm MidPos", ArmMidPos);
        ArmSourcePos = SmartDashboard.getNumber("Arm SourcePos", ArmSourcePos);
        ArmSpeakerClosePos =  SmartDashboard.getNumber("Arm SpeakerClosePos", ArmSpeakerClosePos);
        ArmSpeakerFarPos = SmartDashboard.getNumber("Arm SpeakerFarPos", ArmSpeakerFarPos);
        ArmAmpPos =  SmartDashboard.getNumber("Arm AmpPos", ArmAmpPos);
        arm_MagneticOffset=SmartDashboard.getNumber("Arm Mag Offset", arm_MagneticOffset);
        arm_ForwardSoftLimitThreshold = SmartDashboard.getNumber("Arm ForSLS", arm_ForwardSoftLimitThreshold);
        arm_ReverseSoftLimitThreshold = SmartDashboard.getNumber("Arm RevSLS", arm_ReverseSoftLimitThreshold);
        arm_kA = SmartDashboard.getNumber("Arm kA", arm_kA);
        arm_kP = SmartDashboard.getNumber("Arm kP", arm_kP);
        arm_kI = SmartDashboard.getNumber("Arm kI", arm_kI);
        arm_kD = SmartDashboard.getNumber("Arm kD", arm_kD);
        arm_kG = SmartDashboard.getNumber("Arm kG", arm_kG);
        arm_MMCruiseVelocity = SmartDashboard.getNumber("Arm MM vel", arm_MMCruiseVelocity);
        arm_MMAcceleration = SmartDashboard.getNumber("Arm MM acc", arm_MMAcceleration);
        arm_StatorCurrentLimit = SmartDashboard.getNumber("Arm StatorCurLim", arm_StatorCurrentLimit);
        arm_PeakTC = SmartDashboard.getNumber("Arm Peak TC",arm_PeakTC);
        arm_PeakVolt= SmartDashboard.getNumber("Arm PeakVolt", arm_PeakVolt);


        // ******* UPDATE AUTO CONSTANTS
        autoBuilder_Trans_kP = SmartDashboard.getNumber("Auto AutoBuilder Trans kP", autoBuilder_Trans_kP);
        autoBuilder_Rot_kP =  SmartDashboard.getNumber("Auto AutoBuilder Rot kP", autoBuilder_Rot_kP);
        autoBuilder_Trans_kD = SmartDashboard.getNumber("Auto AutoBuilder Trans kD", autoBuilder_Trans_kD);
        autoBuilder_Rot_kD = SmartDashboard.getNumber("Auto AutoBuilder Rot kD", autoBuilder_Rot_kD);
        constraintVel= SmartDashboard.getNumber("Auto Constraint Vel", constraintVel);
        constraintAcc = SmartDashboard.getNumber("Auto Constraint Acc", constraintAcc);
        constraintRotVel = SmartDashboard.getNumber("Auto Constraint Rot Vel", constraintRotVel);
        constraintRotAcc= SmartDashboard.getNumber("Auto Constraint Rot Acc", constraintRotAcc);


        // ******* UPDATE FOLLOW POSE CONSTANTS
        followPose_Trans_kP = SmartDashboard.getNumber("FollowPose Trans kP",followPose_Trans_kP);
        followPose_Trans_kD = SmartDashboard.getNumber("FollowPose Trans kD",followPose_Trans_kD);
        followPose_Rot_kP = SmartDashboard.getNumber("FollowPose Rot kP",followPose_Rot_kP);
        followPose_Rot_kD = SmartDashboard.getNumber("FollowPose Rot kD",followPose_Rot_kD);
        followPose_Trans_maxV = SmartDashboard.getNumber("FollowPose Trans MaxV", followPose_Trans_maxV);
        followPose_Trans_maxA = SmartDashboard.getNumber("FollowPose Trans MaxA", followPose_Trans_maxA);
        followPose_Rot_maxV = SmartDashboard.getNumber("FollowPose Rot MaxV", followPose_Rot_maxV);
        followPose_Rot_maxA = SmartDashboard.getNumber("FollowPose Rot MaxA", followPose_Rot_maxA);


        // ******* UPDATE DRIVE CONSTANTS
        drive_kP = SmartDashboard.getNumber("Drive kP", drive_kP);
        drive_kI = SmartDashboard.getNumber("Drive kI", drive_kI);
        drive_kD =  SmartDashboard.getNumber("Drive kD", drive_kD);
        drive_kS =  SmartDashboard.getNumber("Drive kS", drive_kS);
        drive_kV = SmartDashboard.getNumber("Drive kV", drive_kV);
        drive_kA = SmartDashboard.getNumber("Drive kA", drive_kA);        


    }
}