package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BumpIntoWall;
import frc.robot.commands.BumpIntoWallReverse;
import frc.robot.commands.CheckForCoralAuto;
import frc.robot.commands.ElevatorCommands.ElevatorToReefA1;
import frc.robot.commands.ElevatorCommands.ElevatorToReefA2;
import frc.robot.commands.ElevatorCommands.ElevatorToReefC1;
import frc.robot.commands.ElevatorCommands.ElevatorToReefC2;
import frc.robot.commands.ElevatorCommands.ElevatorToReefC3;
import frc.robot.commands.ElevatorCommands.ElevatorToReefC4;
import frc.robot.commands.IntakeCommands.IntakeCoralAuto;

public class ActionCommands {
    private CommandSwerveDrivetrain sd;
    private Elevator elevator;
    private Claw claw;
    public Command  elevatorToReef2,elevatorToReef3,
        elevatorToReef4, elevatorToIntake,elevatorToAlgae1,elevatorToAlgae2;
    public Command intakeCoral,spitCoral,bumpIntoWall,elevatorToReef1;
    public Command bumpIntoWallReverse;
    public Command checkForCoralAuto, releaseRamp;

    
    

public  ActionCommands(CommandSwerveDrivetrain m_sd,Elevator m_elevator,Claw m_claw){

    sd=m_sd;
    elevator=m_elevator;
    claw=m_claw;                
        
        elevatorToIntake = new IntakeCoralAuto(m_elevator, m_claw);

        elevatorToReef1 = new ElevatorToReefC1(elevator, claw);
        

        elevatorToReef2 = new ElevatorToReefC2(elevator, claw);

        elevatorToReef3 = new ElevatorToReefC3(elevator, claw);
            
        elevatorToReef4 = new ElevatorToReefC4(elevator, claw);

            
        elevatorToAlgae1 = new ElevatorToReefA1(elevator, claw);
        elevatorToAlgae2 = new ElevatorToReefA2(elevator, claw);

        intakeCoral = new frc.robot.commands.IntakeCommands.IntakeCoral(elevator,claw);

        spitCoral= claw.SpitCoral();

        bumpIntoWall = new BumpIntoWall(sd);
        bumpIntoWallReverse = new BumpIntoWallReverse(sd);
        checkForCoralAuto = new CheckForCoralAuto(claw);
//        releaseRamp = new ReleaseRamp(rearIntake);
        
}
}


