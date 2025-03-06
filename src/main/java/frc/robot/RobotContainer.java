// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// test
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveIntake;
import frc.robot.commands.DriveIntakeRight;
import frc.robot.commands.DriveReefLeft;
import frc.robot.commands.DriveReefRight;
import frc.robot.commands.ElevatorToReef;
import frc.robot.commands.ElevatorToReefC1;
import frc.robot.commands.ElevatorToReefC2;
import frc.robot.commands.ElevatorToReefC3;
import frc.robot.commands.ElevatorToReefC4;
import frc.robot.commands.FollowPoseDirect;
import frc.robot.commands.ElevatorToReefA1;
import frc.robot.commands.ElevatorToReefA2;
import frc.robot.commands.ElevatorToNet;
import frc.robot.commands.ElevatorToProcessor;
import frc.robot.commands.IntakeAlgae1;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ReleaseRamp;
import frc.robot.commands.SpitCoral;
import frc.robot.commands.ZeroElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ActionCommands;
import frc.robot.subsystems.AutoGenerator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RearIntake;
import frc.robot.subsystems.VisionSystem;

public class RobotContainer {
    private Elevator elevator = new Elevator();
    private Claw claw = new Claw();
    private RearIntake rearIntake = new RearIntake();
    

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final RobotJoystick stickDriver = new RobotJoystick(0);
    private final RobotJoystick stickOperator = new RobotJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSystem vision = new VisionSystem(drivetrain);

    public IntakeCoral intakeCoral=new IntakeCoral(elevator, claw);
    public ActionCommands actions = new ActionCommands(drivetrain,elevator,claw,rearIntake );
    public ElevatorToReef elevatorToReef;

    //The auto generator was originally defined just as public, but I changed that, may need to be changed back?
    private final AutoGenerator autoGenerator = new AutoGenerator(elevator, claw, actions);
    private final SendableChooser<Command> autoChooser = autoGenerator.autoChooser;

    public DriveReefLeft driveReefLeft = new DriveReefLeft(drivetrain);
    private Utilities utils = new Utilities(drivetrain, vision);
    public DriveReefRight driveReefRight = new DriveReefRight(drivetrain);
    public DriveIntakeRight driveIntake = new DriveIntakeRight(drivetrain);

    private Trigger receivedCoral;
    private Trigger setSlow;
    

    public RobotContainer() {
        receivedCoral = new Trigger ( ()-> claw.pickedUpCoral() );
        setSlow = new Trigger ( ()-> stickDriver.getRawAxis(3)>0.5);
        configureBindings();        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

// Default drive command        
        drivetrain.setDefaultCommand(
            drivetrain.driveFieldCentricCommand(stickDriver)
        );
        


        //  allow driver to switch between fast and slow mode 
        // changes stick scale factor on joystock


        
//        stickDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
/* 
        stickDriver.button(2).onTrue(new InstantCommand(() 
        -> drivetrain.setTarget(Math.toRadians(-90))));
        stickDriver.button(1).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading( Math.toRadians(54))));
        stickDriver.button(3).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians(-54))));            
        stickDriver.button(4).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians( 0))));   
*/
/*        stickDriver.pov(90).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians( 120))));   
        stickDriver.pov(270).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians( -120))));
            `
        stickDriver.pov(0).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians( 60))));   
        stickDriver.pov(180).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians( -60))));
*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
         
//        (stickDriver.button(1)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
//        (stickDriver.button(2)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
//        (stickDriver.button(3)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
//       (stickDriver.button(4)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        

        // reset the field-centric heading on left bumper press
        stickDriver.button(5).onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyro()));
        stickDriver.button(6).onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyroFlip()));

        setSlow.onTrue(new InstantCommand(() ->stickDriver.setSlowScaleFactor()  )  );
        setSlow.onFalse(new InstantCommand(() ->stickDriver.setFastScaleFactor()  )  );
        
        
//        stickDriver.button(10).onTrue( new InstantCommand(()-> drivetrain.setTrueHeading()));
//        stickDriver.button(3).whileTrue(new DriveReefLeft(drivetrain));                 
//        stickDriver.button(2).whileTrue( new DriveReefRight(drivetrain));                 
//        stickDriver.button(4).whileTrue( new DriveIntake(drivetrain));                 
//        stickDriver.button(4).onTrue(new ReleaseRamp(rearIntake));


        stickOperator.button(1).onTrue(new ElevatorToReefC1(elevator,claw));                     
        stickOperator.button(2).onTrue(new ElevatorToReefC2(elevator,claw));                     
        stickOperator.button(3).onTrue(new ElevatorToReefC3(elevator,claw)); 
        stickOperator.button(4).onTrue(new ElevatorToReefC4(elevator,claw)); 
        
        stickOperator.button(14).onTrue(new ElevatorToReefA1(elevator,claw));                 
        stickOperator.button(13).onTrue(new ElevatorToReefA2(elevator,claw));    
        //stickOperator.button(9).onTrue(new ElevatorToNet(elevator,claw));                                                  

        stickOperator.button(7).onTrue(new IntakeCoral(elevator,claw));

        
        stickOperator.button(9).onTrue
                (new ElevatorToProcessor(elevator,claw));
        stickOperator.button(10).onTrue
                (new ElevatorToNet(elevator,claw));                

        stickOperator.button(6).onTrue(claw.SpitAlgae());
        stickOperator.button(6).onFalse(claw.StopRollers());

        stickOperator.button(5).onTrue(new SpitCoral(claw));

//        receivedCoral.onTrue(claw.StopRollers().andThen(new ElevatorToReefC2(elevator,claw)));
//        receivedCoral.onTrue(claw.StopRollers());

//        stickOperator.button(12).onTrue(new InstantCommand(() -> elevator.elevatorRight.setPosition(0)));

        stickOperator.button(11).onTrue(new InstantCommand( () -> {updateConstants();}));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void updateConstants(){
        claw.updateConstants();
        elevator.updateConstants();
        vision.configure();
//        drivetrain.configure();
     }

     public Command zeroElevator(){
        return (new ZeroElevator(elevator));
     }


     public Command zeroGyro(){
        return (drivetrain.runOnce(() -> drivetrain.zeroGyro()));
     }

     public Command releaseRamp(){
        return (new ReleaseRamp(rearIntake));
     }

    }
