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
    
    
    

public  ActionCommands(CommandSwerveDrivetrain m_sd,Elevator m_elevator,Claw m_claw){

    sd=m_sd;
    elevator=m_elevator;
    claw=m_claw;

    constraints = new PathConstraints(
        1.5, 
        1.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(450));
                
        }

        private Command delay05=new InstantCommand(()->Timer.delay(.5));
        private Command delay1=new InstantCommand(()->Timer.delay(1));

        public Command elevatorToIntake = 
            claw.ToPosition(claw.positionIntake).
            andThen(delay05).
            andThen(elevator.ToPosition(elevator.positionIntake));

        public Command elevatorToReef1 = 
            claw.ToPosition(claw.positionCoral1).
            andThen(delay05).
            andThen(elevator.ToPosition(elevator.positionCoral1));
        

        public Command elevatorToReef2 = 
            claw.ToPosition(claw.positionCoral2).
            andThen(delay05).
            andThen(elevator.ToPosition(elevator.positionCoral2));

        public Command elevatorToReef3 = 
            claw.ToPosition(claw.positionCoral3).
            andThen(delay05).
            andThen(elevator.ToPosition(elevator.positionCoral3));
            
        public Command elevatorToReef4 = 
            claw.ToPosition(claw.positionCoral4).
            andThen(delay05).
            andThen(elevator.ToPosition(elevator.positionCoral4));

            
        public Command elevatorToAlgae1 = 
            claw.ToPosition(claw.positionAlgae1).
            andThen(delay05).
            andThen(elevator.ToPosition(elevator.positionAlgae1));

        public Command elevatorToAlgae2 = 
            claw.ToPosition(claw.positionAlgae2).
            andThen(delay05).
            andThen(elevator.ToPosition(elevator.positionAlgae2));

        public Command intakeCoral = new frc.robot.commands.IntakeCoral(elevator,claw);

        public Command spitCoral=claw.RollersForward().
        andThen(delay1).
        andThen(claw.StopRollers());

        public Command bumpIntoWall = new BumpIntoWall(sd);
}



