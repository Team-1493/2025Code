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
import frc.robot.commands.DriveIntake;
import frc.robot.commands.DriveReefLeft;
import frc.robot.commands.DriveReefRight;
import frc.robot.commands.ElevatorToReef;
import frc.robot.commands.ElevatorToReefC1;
import frc.robot.commands.ElevatorToReefC2;
import frc.robot.commands.ElevatorToReefC3;
import frc.robot.commands.ElevatorToReefC4;

import frc.robot.commands.ElevatorToReefA1;
import frc.robot.commands.ElevatorToReefA2;
import frc.robot.commands.ElevatorToNet;

import frc.robot.commands.IntakeAlgae1;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ZeroElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ActionCommands;
import frc.robot.subsystems.AutoGenerator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.VisionSystem;

public class RobotContainer {
    private Elevator elevator = new Elevator();
    private Claw claw = new Claw();
    

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final RobotJoystick stickDriver = new RobotJoystick(0);
    private final RobotJoystick stickOperator = new RobotJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //public final VisionSystem vision = new VisionSystem(drivetrain);

    public IntakeCoral intakeCoral=new IntakeCoral(elevator, claw);
    public ElevatorToReef elevatorToReef;

    //The auto generator was originally defined just as public, but I changed that, may need to be changed back?
    private final AutoGenerator autoGenerator = new AutoGenerator(elevator, claw);
    private final SendableChooser<Command> autoChooser = autoGenerator.autoChooser;

    public ActionCommands actions = new ActionCommands(drivetrain,elevator,claw); 
    public DriveReefLeft driveReefLeft = new DriveReefLeft(drivetrain);
    public DriveReefRight driveReefRight = new DriveReefRight(drivetrain);
    public DriveIntake driveIntake = new DriveIntake(drivetrain);

    public RobotContainer() {
        drivetrain.setupHeadingController();
        configureBindings();        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

// Default drive command        
        drivetrain.setDefaultCommand(
            drivetrain.driveFieldCentricCommand(stickDriver)
        );
        


/*     stickDriver.button(1).whileTrue(driveReefLeft);
    stickDriver.button(1).onFalse(
        new InstantCommand( ()-> drivetrain.ResetHeadingController()));

    stickDriver.button(2).whileTrue(driveReefRight);
    stickDriver.button(2).onFalse(
            new InstantCommand( ()-> drivetrain.ResetHeadingController()));        
    
    stickDriver.button(3).whileTrue(driveIntake);
    stickDriver.button(3).onFalse(
            new InstantCommand( ()-> drivetrain.ResetHeadingController()));        
  */      


        //  allow driver to switch between fast and slow mode 
        // changes stick scale factor on joystock


        
//        stickDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
//        stickDriver.button(2).onTrue(new InstantCommand(() 
//        -> drivetrain.setTargetHeading(Math.toRadians(-90))  )  );
//        stickDriver.button(1).onTrue(new InstantCommand(() ->
//            drivetrain.setTargetHeading(Math.toRadians( 179.9))  )  );
//        stickDriver.button(3).onTrue(new InstantCommand(() ->
//            drivetrain.setTargetHeading(Math.toRadians(90))  )  );            
//        stickDriver.button(4).onTrue(new InstantCommand(() ->
//            drivetrain.setTargetHeading(Math.toRadians(0))  )  );      


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /* 
        stickDriver.back().and(stickDriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        stickDriver.back().and(stickDriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        stickDriver.start().and(stickDriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        stickDriver.start().and(stickDriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // reset the field-centric heading on left bumper press
        stickDriver.button(5).onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyro()));

        stickDriver.button(6).onTrue(
                new InstantCommand(() ->stickDriver.setSlowScaleFactor()  )  );
        
        stickDriver.button(6).onFalse(
                    new InstantCommand(() ->stickDriver.setFastScaleFactor()  )  );
         
//       stickOperator.button(1).onTrue(claw.ToPosition(-.05));
//        stickOperator.button(2).onTrue(claw.ToPosition(.1));
//        stickOperator.button(3).onTrue(claw.ToPosition(.25));
//        stickOperator.button(4).onTrue(claw.ToPosition(.32));

        stickOperator.button(1).onTrue(new ElevatorToReefC1(elevator,claw));                     
        stickOperator.button(2).onTrue(new ElevatorToReefC2(elevator,claw));                     
        stickOperator.button(3).onTrue(new ElevatorToReefC3(elevator,claw)); 
        stickOperator.button(4).onTrue(new ElevatorToReefC4(elevator,claw)); 
        
        stickOperator.button(14).onTrue(new ElevatorToReefA1(elevator,claw));                 
        stickOperator.button(13).onTrue(new ElevatorToReefA2(elevator,claw));    
        stickOperator.button(9).onTrue(new ElevatorToNet(elevator,claw));                                                  

        stickOperator.button(7).whileTrue(new IntakeCoral(elevator,claw));
        stickOperator.button(7).onFalse(claw.StopRollers());

        stickOperator.button(8).whileTrue
                (new IntakeAlgae1(claw));

        stickOperator.button(6).onTrue(claw.SpitAlgae());
        stickOperator.button(6).onFalse(claw.StopRollers());

        stickOperator.button(5).onTrue(actions.spitCoral);
        stickOperator.button(5).onFalse(claw.StopRollers());


        stickOperator.button(12).onTrue(new InstantCommand(() -> elevator.elevatorRight.setPosition(0)));

        stickOperator.button(11).onTrue(new InstantCommand( () -> {updateConstants();}));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void updateConstants(){
        claw.updateConstants();
        elevator.updateConstants();
//        vision.configure();
//        drivetrain.configure();
     }

     public Command zeroElevator(){
        return (new ZeroElevator(elevator));
     }

    }
