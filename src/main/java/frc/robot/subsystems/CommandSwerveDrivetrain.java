package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Utilities.RobotJoystick;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

//  ***Added to default code!    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    boolean pointInDirection=false;
    private double reverseDirection=1;
    public double yawOffset=0;
    private double accX=0,accY=0,accR=0;
    private double kA_drive=0;
    private ChassisSpeeds speeds_target = new ChassisSpeeds();

private SwerveRequest.RobotCentric driveRC= new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

 
private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
.withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    public static Pose2d robotpose = new Pose2d(0,0,new Rotation2d(0));


    // Simulation stuff
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

 
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(9), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        writeInitialConstants();
        initializeAutoBuilder();
        setLimits();

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }       

        writeInitialConstants();
        initializeAutoBuilder();
        setLimits();
        yawOffset = getYaw();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */

/*          if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
            */

        // added static robotpose  so AprilTagCamera can get the pose for simulation
        robotpose=this.getPose();
        SmartDashboard.putNumber("Pose X",robotpose.getX());
        SmartDashboard.putNumber("Pose Y",robotpose.getY());
        SmartDashboard.putNumber("Pose Z",robotpose.getRotation().getDegrees());
        SmartDashboard.putNumber("TrueHeading", getTrueHeading()*180/Math.PI);
        SmartDashboard.putNumber("vX_target", speeds_target.vxMetersPerSecond);
        SmartDashboard.putNumber("vY_target", speeds_target.vyMetersPerSecond);
        SmartDashboard.putNumber("vZ_target", speeds_target.omegaRadiansPerSecond);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }



// DriveRobotCentric is used for autos and april tag following
// x and y are speeds in m/s,  z is rotational rate in rad/sec    
    public Command driveRobotCentricCommand(double x,double y,double z) {
        return runOnce(() -> this.driveRobotCentric(x,y,z));
    }

    public  void driveRobotCentric(ChassisSpeeds speeds) {
        calculateAcceleration(speeds);

        this.setControl(driveRC.
        withVelocityX(speeds.vxMetersPerSecond+accX*kA_drive)
        .withVelocityY(speeds.vyMetersPerSecond+accY*kA_drive)
        .withRotationalRate(speeds.omegaRadiansPerSecond)
        );


}

    public  void driveRobotCentric(double x,double y,double z) {
            calculateAcceleration(x,y,z);
            speeds_target= new ChassisSpeeds(x+accX*kA_drive,y+accY*kA_drive,z);
            this.setControl(driveRC.
            withVelocityX(x+accX*kA_drive)
            .withVelocityY(y+accY*kA_drive)
            .withRotationalRate(z)
            );
    }

// DriveFieldCentric  is used for teleop with joystick control
// x, y, and z are joystick inputs from -1 to 1
// x and y are speeds in m/s,  z is rotational rate in rad/sec    
    public Command driveFieldCentricCommand(RobotJoystick stickDriver) {
        return runOnce(() -> this.driveFieldCentric(stickDriver));
    }

    public  void driveFieldCentric(RobotJoystick stickDriver) {
        double x = -stickDriver.getY2()*MaxSpeed*reverseDirection;
        double y = -stickDriver.getX2()*MaxSpeed*reverseDirection;
        double z = -stickDriver.getRotate()*MaxAngularRate; 
        driveFieldCentric(x, y, z);
   

    }


    public  void driveFieldCentric(double x,double y, double z) {
        calculateAcceleration(x,y,z);

//        System.out.println("******   "+
//        this.getModule(0).getDriveMotor().getTorqueCurrent().getValueAsDouble()
//        +"     "+ x + "    "+accX*20
 //       );

        speeds_target= new ChassisSpeeds(x+accX*kA_drive,y+accY*kA_drive,z);

        this.setControl(driveFC
        .withVelocityX(x+accX*kA_drive)
        .withVelocityY(y+accY*kA_drive)
        .withRotationalRate(z));
    }
    
    public void resetToFieldZero(){
        this.resetRotation(Rotation2d.fromRadians(getYaw()-yawOffset) );
    }

    public void setRotationToZero(){
        this.resetRotation(Rotation2d.fromRadians(0));
    }

    public void setFieldZero(){
        yawOffset = getYaw();
        setRotationToZero();
    }

    private double getTrueHeading(){
        double th = getYaw()-yawOffset;
        if (th > Math.PI) th=th-2*Math.PI;
        else if (th < -Math.PI) th = th + 2*Math.PI;
        return th;
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public double getYaw() {
        return (this.getPigeon2().getYaw().getValueAsDouble()*Math.PI/180.0);
    }    

    public ChassisSpeeds getChassisSpeed(){
        
        return getState().Speeds;
    }


    private void calculateAcceleration(double vx,double vy, double vr){
        accX = (vx-this.getState().Speeds.vxMetersPerSecond)/0.02;
        accY = (vy - this.getState().Speeds.vyMetersPerSecond)/0.02;
        accR = (vr - this.getState().Speeds.omegaRadiansPerSecond)/0.02;

        
    }

    private void calculateAcceleration(ChassisSpeeds speeds){
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double vr = speeds.omegaRadiansPerSecond;
        // ****** had error wiht (), fixed
        accX = (vx-this.getState().Speeds.vxMetersPerSecond)/0.02;
        accY = (vy - this.getState().Speeds.vyMetersPerSecond)/0.02;
        accR = (vr - this.getState().Speeds.omegaRadiansPerSecond)/0.02;
    }



    public void initializeAutoBuilder(){
        RobotConfig config;
        try{
            double kPAuto=SmartDashboard.getNumber("Drive Auto kP", 0);
            double kDAuto=SmartDashboard.getNumber("Drive Auto kD", 0);
            double kProtAuto=SmartDashboard.getNumber("Drive Auto rot kP", 0);
            double kDrotAuto=SmartDashboard.getNumber("Drive Auto rot kD", 0);
            config = RobotConfig.fromGUISettings();



            AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeed,
            (speeds, feedforwards) -> driveRobotCentric(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(kPAuto, 0.0, kDAuto),
                new PIDConstants(kProtAuto, 0.0, kDrotAuto)
            ),
            config,
            /*() -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Blue;  
                }
                return false;
            }*/
            () -> {return false;},
            this
        );
        } catch (Exception e) {
        }
    }

    public void writeInitialConstants(){

        // Default constants for open loop voltage control
        SmartDashboard.putNumber("Drive kP",0.11967);  // 6.5 for TC,0.1 Voltage
        SmartDashboard.putNumber("Drive kV",0.1226);  // 0 for TC,0.124 Voltage
        SmartDashboard.putNumber("Drive kA",0);  // 0.08826 for TC
        SmartDashboard.putNumber("Drive kS",0.16936);  // 0 for TC


        SmartDashboard.putNumber("Drive Auto kP",4); 
        SmartDashboard.putNumber("Drive Auto kD",0);         
        SmartDashboard.putNumber("Drive Auto rot kP", 4);
        SmartDashboard.putNumber("Drive Auto rot kD", 0);



    }


    public void configure(){
        Slot0Configs slot0config = new Slot0Configs();
        slot0config.kP=SmartDashboard.getNumber("Drive kP", 0);
        slot0config.kV=SmartDashboard.getNumber("Drive kV", 0);
        kA_drive=SmartDashboard.getNumber("Drive kA", 0);
        slot0config.kA=kA_drive;
        slot0config.kS=SmartDashboard.getNumber("Drive kS", 0);


        this.getModule(0).getDriveMotor().getConfigurator().apply(slot0config);
        this.getModule(1).getDriveMotor().getConfigurator().apply(slot0config);
        this.getModule(2).getDriveMotor().getConfigurator().apply(slot0config);
        this.getModule(3).getDriveMotor().getConfigurator().apply(slot0config);
     
        initializeAutoBuilder();

        

    }

    public Command Stop(){
        return driveRobotCentricCommand(0, 0, 0);
    }

    public void stop(){
        driveRobotCentricCommand(0, 0, 0);
    }

    private void setLimits(){
        CurrentLimitsConfigs clc = new CurrentLimitsConfigs();
        clc.StatorCurrentLimit=60;
        clc.SupplyCurrentLimit=40;
        clc.StatorCurrentLimitEnable=true;


        this.getModule(0).getDriveMotor().getConfigurator().apply(clc);
        this.getModule(1).getDriveMotor().getConfigurator().apply(clc);
        this.getModule(2).getDriveMotor().getConfigurator().apply(clc);
        this.getModule(3).getDriveMotor().getConfigurator().apply(clc);
        this.getModule(0).getSteerMotor().getConfigurator().apply(clc);
        this.getModule(1).getSteerMotor().getConfigurator().apply(clc);
        this.getModule(2).getSteerMotor().getConfigurator().apply(clc);
        this.getModule(3).getSteerMotor().getConfigurator().apply(clc);

    }


}