package frc.robot.subsystems;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BumpIntoWall;
import frc.robot.commands.BumpIntoWallReverse;
import frc.robot.commands.CheckForCoralAuto;
import frc.robot.commands.ElevatorToReefA1;
import frc.robot.commands.ElevatorToReefA2;
import frc.robot.commands.ElevatorToReefC1;
import frc.robot.commands.ElevatorToReefC2;
import frc.robot.commands.ElevatorToReefC3;
import frc.robot.commands.ElevatorToReefC4;
import frc.robot.commands.IntakeCoralAuto;
import frc.robot.commands.ReleaseRamp;

public class ActionCommands {

    static private PathConstraints constraints;
    private CommandSwerveDrivetrain sd;
    private Elevator elevator;
    private Claw claw;
    public Command  elevatorToReef2,elevatorToReef3,
        elevatorToReef4, elevatorToIntake,elevatorToAlgae1,elevatorToAlgae2;
    public Command intakeCoral,spitCoral,bumpIntoWall,elevatorToReef1;
    public Command bumpIntoWallReverse;
    public Command checkForCoralAuto, releaseRamp;

    
    

public  ActionCommands(CommandSwerveDrivetrain m_sd,Elevator m_elevator,Claw m_claw,
            RearIntake rearIntake){

    sd=m_sd;
    elevator=m_elevator;
    claw=m_claw;

    constraints = new PathConstraints(
        1.5, 
        1.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(450));
                
        
        elevatorToIntake = new IntakeCoralAuto(m_elevator, m_claw);

        elevatorToReef1 = new ElevatorToReefC1(elevator, claw);
        

        elevatorToReef2 = new ElevatorToReefC2(elevator, claw);

        elevatorToReef3 = new ElevatorToReefC3(elevator, claw);
            
        elevatorToReef4 = new ElevatorToReefC4(elevator, claw);

            
        elevatorToAlgae1 = new ElevatorToReefA1(elevator, claw);
        elevatorToAlgae2 = new ElevatorToReefA2(elevator, claw);

        intakeCoral = new frc.robot.commands.IntakeCoral(elevator,claw);

        spitCoral= claw.SpitCoral();

        bumpIntoWall = new BumpIntoWall(sd);
        bumpIntoWallReverse = new BumpIntoWallReverse(sd);
        checkForCoralAuto = new CheckForCoralAuto(claw);
        releaseRamp = new ReleaseRamp(rearIntake);
        
}
}


