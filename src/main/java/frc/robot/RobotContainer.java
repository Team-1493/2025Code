// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// test
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Utilities.RobotJoystick;
import frc.robot.Utilities.Telemetry;
import frc.robot.Utilities.VisionConstants;
import frc.robot.commands.DriveToCommands2;
import frc.robot.commands.ReleaseRamp;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ElevatorCommands.ElevatorToNet;
import frc.robot.commands.ElevatorCommands.ElevatorToProcessor;
import frc.robot.commands.ElevatorCommands.ElevatorToReef;
import frc.robot.commands.ElevatorCommands.ElevatorToReefA1;
import frc.robot.commands.ElevatorCommands.ElevatorToReefA2;
import frc.robot.commands.ElevatorCommands.ElevatorToReefC1;
import frc.robot.commands.ElevatorCommands.ElevatorToReefC2;
import frc.robot.commands.ElevatorCommands.ElevatorToReefC3;
import frc.robot.commands.ElevatorCommands.ElevatorToReefC4;
import frc.robot.commands.IntakeCommands.IntakeCoral;
import frc.robot.commands.SpitCommands.SpitCoral;
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
    public Claw claw = new Claw();
    private RearIntake rearIntake = new RearIntake();
    

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final RobotJoystick stickDriver = new RobotJoystick(0);
    private final RobotJoystick stickOperator = new RobotJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public IntakeCoral intakeCoral=new IntakeCoral(elevator, claw);
    public ActionCommands actions = new ActionCommands(drivetrain,elevator,claw );
    public ElevatorToReef elevatorToReef;

    //The auto generator was originally defined just as public, but I changed that, may need to be changed back?
    private final AutoGenerator autoGenerator = new AutoGenerator(elevator, claw, actions);
    private final SendableChooser<Command> autoChooser = autoGenerator.autoChooser;
    public final VisionConstants visionConstants = new VisionConstants();
    public final VisionSystem vision = new VisionSystem(drivetrain);


    public DriveToCommands2 driveToCommands = new DriveToCommands2(drivetrain);

    private Trigger setSlow;

    

    public RobotContainer() {

        setSlow = new Trigger ( ()-> stickDriver.getRawAxis(3)>0.5);

        configureBindings();        
    }

    private void configureBindings() {

// Default drive command        
        drivetrain.setDefaultCommand(
            drivetrain.driveFieldCentricCommand(stickDriver)
        );
        
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
         
//        (stickDriver.button(3)).whileTrue(claw.sysIdDynamic(Direction.kForward));
//        (stickDriver.button(4)).whileTrue(claw.sysIdDynamic(Direction.kReverse));
//        (stickDriver.button(1)).whileTrue(claw.sysIdQuasistatic(Direction.kForward));
//        (stickDriver.button(2)).whileTrue(claw.sysIdQuasistatic(Direction.kReverse));
//       stickDriver.button(4).onFalse((new InstantCommand( ()-> new WaitCommand(5) )).
//                  andThen(new InstantCommand( ()->SignalLogger.stop() )));
        


//        stickDriver.button(1).onTrue(claw.ToPosition(-0.05));
//        stickDriver.button(2).onTrue(claw.ToPosition(0.32));
//        stickDriver.button(4).onTrue(claw.StopClaw());



//        stickDriver.button(5).whileTrue( claw.ClawUp());
//        stickDriver.button(5).onFalse( claw.StopClaw());
//        stickDriver.button(6).whileTrue( claw.ClawDown());
//        stickDriver.button(6).onFalse( claw.StopClaw());

    
        stickDriver.button(5).onTrue( new InstantCommand(()-> drivetrain.setRotationToZero()));
        stickDriver.button(6).onTrue( new InstantCommand(()-> drivetrain.resetToFieldZero()));
        stickDriver.button(7).onTrue( new InstantCommand(()-> drivetrain.setFieldZero()));

        setSlow.onTrue(new InstantCommand(() ->stickDriver.setSlowScaleFactor()  )  );
        setSlow.onFalse(new InstantCommand(() ->stickDriver.setFastScaleFactor()  )  );

  
        stickDriver.button(3).whileTrue( new DeferredCommand( 
            () -> driveToCommands.getCommandLeft() , Set.of(drivetrain)));

        stickDriver.button(2).whileTrue( new DeferredCommand( 
            () -> driveToCommands.getCommandRight() , Set.of(drivetrain)));

        stickDriver.button(4).whileTrue( new DeferredCommand( 
            () -> driveToCommands.getCommandCenter() , Set.of(drivetrain)));




//        stickDriver.button(1).whileTrue( new DeferredCommand( 
//            () -> driveToCommands.getIntakeCommand() , Set.of(drivetrain)));
   
//        stickDriver.pov(180).whileTrue( new DeferredCommand( 
//            () -> driveToCommands.getProcessorCommand() , Set.of(drivetrain)));
        



        stickOperator.button(1).onTrue(new ElevatorToReefC1(elevator,claw));                     
        stickOperator.button(2).onTrue(new ElevatorToReefC2(elevator,claw));                     
        stickOperator.button(3).onTrue(new ElevatorToReefC3(elevator,claw)); 
        stickOperator.button(4).onTrue(new ElevatorToReefC4(elevator,claw)); 

        stickOperator.button(14).onTrue(new ElevatorToReefA1(elevator,claw));                 
        stickOperator.button(13).onTrue(new ElevatorToReefA2(elevator,claw));  

    
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

        stickOperator.button(12).onTrue(new InstantCommand(() -> elevator.elevatorRight.setPosition(0)));
//        stickOperator.button(11).whileTrue(elevator.ManualDown());
//        stickOperator.button(11).onFalse(elevator.StopElevator());


        stickOperator.button(11).onTrue(new InstantCommand( () -> {updateConstants();}));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void updateConstants(){
//        claw.updateConstants();
//        elevator.updateConstants();
//        vision.configure();
        drivetrain.configure();
     }

     public Command zeroElevator(){
        return (new ZeroElevator(elevator));
     }



     public Command releaseRamp(){
        return (new ReleaseRamp(rearIntake));
     }

    }
