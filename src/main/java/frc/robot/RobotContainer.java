// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// test
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private Elevator elevator = new Elevator();
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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
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
        


// Driver Joystick Bindings


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
        stickDriver.back().and(stickDriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        stickDriver.back().and(stickDriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        stickDriver.start().and(stickDriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        stickDriver.start().and(stickDriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        stickDriver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // manual control of elevator          
        stickDriver.povDown().whileTrue(elevator.manualDownCommand());
        stickDriver.povDown().onFalse(elevator.stopElevatorCommand());
        stickDriver.povUp().whileTrue(elevator.manualUpCommand());
        stickDriver.povUp().onFalse(elevator.stopElevatorCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
