// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// test
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagCam;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

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
    public final AprilTagCam aprilTagCam = new AprilTagCam();

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
        


    //***& Driver Joystick Bindings ***


        //  allow driver to switch between fast and slow mode 
        // changes stick scale factor on joystock
        stickDriver.axisGreaterThan(3, .1).onTrue(
            new InstantCommand(() ->stickDriver.setSlowScaleFactor()  )  );
    
        stickDriver.axisGreaterThan(3, .1).onFalse(
                new InstantCommand(() ->stickDriver.setSlowScaleFactor()  )  );

        
//        stickDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        stickDriver.button(2).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians(-90))  )  );
        stickDriver.button(1).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians( 179.9))  )  );
        stickDriver.button(3).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians(90))  )  );            
        stickDriver.button(4).onTrue(new InstantCommand(() ->
            drivetrain.setTargetHeading(Math.toRadians(0))  )  );      


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /* 
        stickDriver.back().and(stickDriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        stickDriver.back().and(stickDriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        stickDriver.start().and(stickDriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        stickDriver.start().and(stickDriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // reset the field-centric heading on left bumper press
        stickDriver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // *** Operator Joystick Bindings ***

        // manual control of elevator          
        stickOperator.button(9).whileTrue(elevator.manualDownCommand());
        stickOperator.button(9).onFalse(elevator.stopElevatorCommand());
        stickOperator.button(10).whileTrue(elevator.manualUpCommand());
        stickOperator.button(10).onFalse(elevator.stopElevatorCommand());


        // elevator to preset positions
//        stickDriver.povUp().onTrue(elevator.toPositionCommand(elevator.pos5));
//        stickDriver.povDown().onTrue(elevator.toPositionCommand(elevator.pos1));
//        stickDriver.povRight().onTrue(elevator.toPositionCommand(elevator.pos2));
//        stickDriver.povLeft().onTrue(elevator.toPositionCommand(elevator.pos3));


        // manual control of claw          
        stickOperator.button(7).whileTrue(claw.manualDownCommand());
        stickOperator.button(8).onFalse(claw.stopClawCommand());
        stickOperator.button(7).whileTrue(claw.manualUpCommand());
        stickOperator.button(8).onFalse(claw.stopClawCommand());

         // claw to preset positions
//         stickDriver.button(1).onTrue(claw.toPositionCommand(claw.pos1));
//         stickDriver.button(2).onTrue(claw.toPositionCommand(claw.pos2));

        // manual control of rollers          
        stickOperator.button(3).whileTrue(claw.manualFrontRollerForCommand());
        stickOperator.button(3).onFalse(claw.stopRollerFrontCommand());
        stickOperator.button(4).whileTrue(claw.manualRearRollerForCommand());
        stickOperator.button(4).onFalse(claw.stopRollerRearCommand());


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
