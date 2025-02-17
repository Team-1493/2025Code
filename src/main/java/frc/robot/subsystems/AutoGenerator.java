package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCoral;

public class AutoGenerator {
    public SendableChooser<Command> autoChooser;
    Elevator elevator;
    Claw claw;

    private void defineCommands(Command command0, Command command1, Command command2, Command command3, Command simIntakeCoral, Command simIntakeAlgae, Command simScoreCoral, Command simDropAlgae){
        NamedCommands.registerCommand("testAction0", command0);
        NamedCommands.registerCommand("testAction1", command1);
        NamedCommands.registerCommand("testAction2", command2);
        NamedCommands.registerCommand("testAction3", command3);

        NamedCommands.registerCommand("simIntakeCoral", simIntakeCoral);
        NamedCommands.registerCommand("simIntakeAlgae", simIntakeAlgae);
        NamedCommands.registerCommand("simScoreCoral", simScoreCoral);
        NamedCommands.registerCommand("simDropAlgae", simDropAlgae);
    }

    private void autoChooserInit(){
        autoChooser = AutoBuilder.buildAutoChooser("Auto1");

        SmartDashboard.putBoolean("autocommand0Successful", false);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        //Variables for simulating 3 Coral Auto
        SmartDashboard.putBoolean("Holding Coral", true);
        SmartDashboard.putBoolean("Holding Algae", false);
        SmartDashboard.putNumber("Scored Coral", 0);

    }


    public AutoGenerator( Elevator m_elevator, Claw m_claw){
        elevator = m_elevator;
        claw = m_claw;

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
   
        defineCommands(command0, command1, command2, command3, simIntakeCoral, simIntakeAlgae, simScoreCoral, simDropAlgae);
        
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


}
