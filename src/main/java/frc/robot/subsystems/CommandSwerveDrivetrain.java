package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotJoystick;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

//  ***Added to default code!    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double targetHeading=0, targetHeadingPrev=0,headingRateDeadband=1;
    boolean pointInDirection=false;

    private TrapezoidProfile.Constraints tp =
         new TrapezoidProfile.Constraints(3, 6);
    private ProfiledPIDController headingController = new ProfiledPIDController(20, 0, 0,tp);

    private SwerveRequest.RobotCentric driveRC= new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);;

    private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    static Pose2d robotpose = new Pose2d(0,0,new Rotation2d(0));

 
    // States:  
    //      driving cam guided to reef = 1. use only front cams  
    //      driving cam guided to intake = 2, use only rear cams
    //      other = 3, use both cams

    public int camState = 0;

    // Simulation stuff
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
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
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // added static robotpose  so AprilTagCamera can get the pose for simulation
        robotpose=this.getPose();
        SmartDashboard.putNumber("Pose X",robotpose.getX());
        SmartDashboard.putNumber("Pose Y",robotpose.getY());
        SmartDashboard.putNumber("Pose Z",robotpose.getRotation().getDegrees());

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
        this.setControl(driveRC.withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
}

    public  void driveRobotCentric(double x,double y,double z) {
            this.setControl(driveRC.withVelocityX(x)
            .withVelocityY(y)
            .withRotationalRate(z));
    }

// DriveFieldCentric  is used for teleop with joystick control
// x, y, and z are joystick inputs from -1 to 1
// x and y are speeds in m/s,  z is rotational rate in rad/sec    
    public Command driveFieldCentricCommand(RobotJoystick stickDriver) {
        return runOnce(() -> this.driveFieldCentric(stickDriver));
    }

    public  void driveFieldCentric(RobotJoystick stickDriver) {
        double x = -stickDriver.getY2()*MaxSpeed;
        double y = -stickDriver.getX2()*MaxSpeed;
        double z = -stickDriver.getRotate()*MaxAngularRate;

        double headingRate=this.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        headingRate=Math.toRadians(headingRate);
        SmartDashboard.putNumber("Drive HeadingRate", headingRate);
        double heading=this.getPose().getRotation().getRadians();
        SmartDashboard.putNumber("Drive Heading", heading);


        if(Math.abs(z)>0 || (Math.abs(headingRate)>headingRateDeadband && !pointInDirection)) {
            targetHeadingPrev=targetHeading;
            targetHeading=heading; 
            pointInDirection=false;
            SmartDashboard.putNumber("TargetHeading", targetHeading);
        }
        else {  if (!pointInDirection && Math.abs(headingRate)<headingRateDeadband) {
                    pointInDirection=true;
                    targetHeading=heading;
                    targetHeadingPrev=targetHeading;
                    resetHeadingController(targetHeading);

                }
                headingRate = headingController.calculate(heading);  
                if (headingController.atSetpoint()) headingRate=0;
                z=headingRate;
                SmartDashboard.putNumber("TargetHeading", targetHeading);
        }

        driveFieldCentric(x, y, z);
    }


    public  void driveFieldCentric(double x,double y, double z) {
        this.setControl(driveFC
        .withVelocityX(x)
        .withVelocityY(y)
        .withRotationalRate(z));
    }


    public void setTargetHeading(double heading){
        SmartDashboard.putNumber("TargetHeading", heading);
        targetHeading=heading;
        pointInDirection=true;  
        SmartDashboard.putBoolean("HeadingControl", pointInDirection);
        headingController.setGoal(targetHeading); 
    }

    public void resetHeadingController(double angle){
        headingController.reset(angle);
        targetHeading=angle;
        targetHeadingPrev=targetHeading;
        headingController.setGoal(targetHeading);
      }

      public void resetHeadingController(){
        resetHeadingController
        (this.getPose().getRotation().getRadians());
    }
      
      public Command ResetHeadingController(){
        return runOnce( () -> {  resetHeadingController
            (this.getPose().getRotation().getRadians()); });
      }

    public void setupHeadingController(){
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(0.017);  //1 degree
        targetHeading =0;// this.getPose().getRotation().getRadians();
        headingController.setGoal(0);      

    }

    public void zeroGyro(){
        seedFieldCentric();
        resetHeadingController(this.getPose().getRotation().getRadians());        

    }


    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public ChassisSpeeds getChassisSpeed(){
        return getState().Speeds;
    }


    public void initializeAutoBuilder(){
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeed,
            (speeds, feedforwards) -> driveRobotCentric(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;  
                }
                return false;
            },
            this
        );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    public void writeInitialConstants(){

        // Default constants for open loop voltage control
        SmartDashboard.putNumber("Drive kP",0.1);  // 3 for TC
        SmartDashboard.putNumber("Drive kV",0.124);  // 0 for TC
        SmartDashboard.putNumber("Drive kA",0.0);  // 0 for TC


    }


    public void updateConstants(){
        Slot0Configs slot0config = new Slot0Configs();
        slot0config.kP=SmartDashboard.getNumber("Drive kP", 0);
        slot0config.kV=SmartDashboard.getNumber("Drive kV", 0);
        slot0config.kA=SmartDashboard.getNumber("Drive kA", 0);
        this.getModule(0).getDriveMotor().getConfigurator().apply(slot0config);
        this.getModule(1).getDriveMotor().getConfigurator().apply(slot0config);
        this.getModule(2).getDriveMotor().getConfigurator().apply(slot0config);
        this.getModule(3).getDriveMotor().getConfigurator().apply(slot0config);
     
    }

    public Command Stop(){
        return driveRobotCentricCommand(0, 0, 0);
    }

    public void stop(){
        driveRobotCentricCommand(0, 0, 0);
    }

}

