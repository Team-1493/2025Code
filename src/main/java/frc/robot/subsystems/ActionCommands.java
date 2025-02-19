package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.VisionConstants;
import frc.robot.commands.BumpIntoWall;

public class ActionCommands {

    static private PathConstraints constraints;
    private CommandSwerveDrivetrain sd;
    private Elevator elevator;
    private Claw claw;
    public SequentialCommandGroup elevatorToReef1, elevatorToReef2,elevatorToReef3,
        elevatorToReef4, elevatorToIntake,elevatorToAlgae1,elevatorToAlgae2;
    public  SequentialCommandGroup spitCoral;
    public Command intakeCoral,bumpIntoWall;


    
    

public  ActionCommands(CommandSwerveDrivetrain m_sd,Elevator m_elevator,Claw m_claw){

    sd=m_sd;
    elevator=m_elevator;
    claw=m_claw;

    constraints = new PathConstraints(
        1.5, 
        1.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(450));
                
        
        elevatorToIntake = new SequentialCommandGroup( 
            claw.ToPosition(claw.positionIntake), 
            new InstantCommand(()->Timer.delay(.5)),
            elevator.ToPosition(elevator.positionIntake));

        elevatorToReef1 = new SequentialCommandGroup( 
            claw.ToPosition(claw.positionCoral1),
            new InstantCommand(()->Timer.delay(.5)), 
            elevator.ToPosition(elevator.positionCoral1));
        

        elevatorToReef2 = new SequentialCommandGroup( 
            claw.ToPosition(claw.positionCoral2),
            new InstantCommand(()->Timer.delay(.5)),
             elevator.ToPosition(elevator.positionCoral2));

        elevatorToReef3 = new SequentialCommandGroup( 
            claw.ToPosition(claw.positionCoral3),
            new InstantCommand(()->Timer.delay(.5)), 
            elevator.ToPosition(elevator.positionCoral3));
            
        elevatorToReef4 = new SequentialCommandGroup( 
            claw.ToPosition(claw.positionCoral4),
            new InstantCommand(()->Timer.delay(.5)),
            elevator.ToPosition(elevator.positionCoral4));

            
        elevatorToAlgae1 = new SequentialCommandGroup( 
            claw.ToPosition(claw.positionAlgae1),
            new InstantCommand(()->Timer.delay(.5)), 
            elevator.ToPosition(elevator.positionAlgae1));

        elevatorToAlgae2 = new SequentialCommandGroup( 
            claw.ToPosition(claw.positionAlgae2),
            new InstantCommand(()->Timer.delay(.5)),
            elevator.ToPosition(elevator.positionAlgae2));

        intakeCoral = new frc.robot.commands.IntakeCoral(elevator,claw);

        spitCoral= new SequentialCommandGroup(
        claw.RollersForward(),
        new InstantCommand(()->Timer.delay(1.5)),claw.StopRollers());

        bumpIntoWall = new BumpIntoWall(sd);
}
}


