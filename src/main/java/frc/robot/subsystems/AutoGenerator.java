package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.ActionCommands;

public class AutoGenerator {
    public SendableChooser<Command> autoChooser;
    Elevator elevator;
    Claw claw;
    ActionCommands actions;
    private void defineCommands(Command simIntakeCoral, Command simIntakeAlgae, 
    Command simScoreCoral, 
    Command simDropAlgae, Command toReef1, Command toReef2, 
    Command toReef3, Command toReef4, Command toIntake, 
    Command driveToReefWall, Command driveToIntakeWall, 
    Command intakeCoral, Command spitCoral, Command  checkForCoralAuto,
    Command autoComplete, Command releaseRamp){
        NamedCommands.registerCommand("simIntakeCoral", simIntakeCoral);
        NamedCommands.registerCommand("simIntakeAlgae", simIntakeAlgae);
        NamedCommands.registerCommand("simScoreCoral", simScoreCoral);
        NamedCommands.registerCommand("simDropAlgae", simDropAlgae);

        NamedCommands.registerCommand("toReef1", toReef1);
        NamedCommands.registerCommand("toReef2", toReef2);
        NamedCommands.registerCommand("toReef3", toReef3);
        NamedCommands.registerCommand("toReef4", toReef4);
        NamedCommands.registerCommand("toIntake", toIntake);
        NamedCommands.registerCommand("driveToReefWall", driveToReefWall);
        NamedCommands.registerCommand("driveToIntakeWall", driveToIntakeWall);
        NamedCommands.registerCommand("intakeCoral", intakeCoral);
        NamedCommands.registerCommand("spitCoral", spitCoral);
        NamedCommands.registerCommand("autoComplete", autoComplete);
        NamedCommands.registerCommand("releaseRamp", releaseRamp);
    }

    private void autoChooserInit(){
        autoChooser = AutoBuilder.buildAutoChooser("Auto1");

        SmartDashboard.putBoolean("autocommand0Successful", false);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        //Variables for simulating Autos
        SmartDashboard.putString("autoSim elevatorGoal", "start");
        SmartDashboard.putBoolean("autoSim holdingCoral", true);
        SmartDashboard.putNumber("autoSim coralScored", 0);
        SmartDashboard.putBoolean("autoSim autoComplete", false);

    }


    public AutoGenerator( Elevator m_elevator, Claw m_claw, ActionCommands m_actions){
        elevator = m_elevator;
        claw = m_claw;
        actions = m_actions;
        

        InstantCommand simIntakeCoral = new InstantCommand(
            () -> SmartDashboard.putBoolean("Holding Coral", true)
        );

        InstantCommand simIntakeAlgae = new InstantCommand(
            () -> SmartDashboard.putBoolean("Holding Algae", true)
        );

        SequentialCommandGroup simScoreCoral = new SequentialCommandGroup(
            new InstantCommand(() -> SmartDashboard.putBoolean("Holding Coral", false)),
            new InstantCommand(() -> SmartDashboard.putNumber("Scored Coral", SmartDashboard.getNumber("Scored Coral", 0)+1))
        );

        InstantCommand simDropAlgae = new InstantCommand(
            () -> SmartDashboard.putBoolean("Holding Algae", false)
        );


        InstantCommand command0 = new InstantCommand(
            () -> SmartDashboard.putBoolean("autocommand0Successful", true)
        );

       InstantCommand command1 = new InstantCommand(                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        () ->m_elevator.toPosition(m_elevator.positionCoral1));  

        Command command2 = new IntakeCoral(elevator,claw);
        
        InstantCommand command3 = new InstantCommand(
            () ->m_claw.toPosition(m_claw.positionCoral1));  
            
        Command auto1 = AutoCommand1();
        
        Command toReef1 = m_actions.elevatorToReef1;
        Command toReef2 = m_actions.elevatorToReef2;
        Command toReef3 = m_actions.elevatorToReef3;
        //SequentialCommandGroup toReef4 = m_actions.elevatorToReef4;
        //autoSim version
        SequentialCommandGroup toReef4 = new SequentialCommandGroup(
            new InstantCommand(() -> SmartDashboard.putString("autoSim elevatorGoal", "beginTo4")),
            m_actions.elevatorToReef4,
            new InstantCommand(() -> SmartDashboard.putString("autoSim elevatorGoal", "endTo4"))
        );


        //SequentialCommandGroup toIntake = m_actions.elevatorToIntake;
        //autoSim version
        SequentialCommandGroup toIntake = new SequentialCommandGroup(
            new InstantCommand(() -> SmartDashboard.putString("autoSim elevatorGoal", "beginToIntake")),
            m_actions.elevatorToIntake,
            new InstantCommand(() -> SmartDashboard.putString("autoSim elevatorGoal", "endToIntake"))
        );

        Command checkForCoralAuto = m_actions.checkForCoralAuto;
        
        Command driveToReefWall = m_actions.bumpIntoWall;
        Command driveToIntakeWall = m_actions.bumpIntoWallReverse;

        //Command intakeCoral = m_actions.intakeCoral;
        //autoSim version
        SequentialCommandGroup intakeCoral = new SequentialCommandGroup(
            new InstantCommand(() -> SmartDashboard.putBoolean("autoSim holdingCoral", true)),
            m_actions.intakeCoral
        );

        SequentialCommandGroup spitCoral = new SequentialCommandGroup(
            m_actions.spitCoral
        );

        InstantCommand autoComplete = new InstantCommand(() -> SmartDashboard.putBoolean("autoSim autoComplete", true));

        Command releaseRamp = m_actions.releaseRamp;

        defineCommands(simIntakeCoral, simIntakeAlgae, simScoreCoral, simDropAlgae, toReef1, toReef2, toReef3, toReef4, toIntake, driveToReefWall, driveToIntakeWall, intakeCoral, spitCoral,
        checkForCoralAuto, autoComplete, releaseRamp);



        autoChooserInit();    
        autoChooser.addOption("Auto1", auto1);
        autoChooser.addOption("Command1", command1);

    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          


     public Command AutoCommand1() {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }


  public Command IntakeR_DR() {
   return new PathPlannerAuto("IntakeR_DR");
  }


  
  public Command Straight2m() {
    return new PathPlannerAuto("Straight2m");
   }
 

}