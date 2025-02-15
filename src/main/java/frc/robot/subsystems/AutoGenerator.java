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
import frc.robot.commands.IntakeCoral;

public class AutoGenerator {
    public SendableChooser<Command> autoChooser;
    Elevator elevator;
    Claw claw;

    private void defineCommands(Command command1, Command command2, Command command3){
        NamedCommands.registerCommand("testAction", command1);
        NamedCommands.registerCommand("testAction", command2);
        NamedCommands.registerCommand("testAction", command3);
    }

    private void autoChooserInit(){
        autoChooser = AutoBuilder.buildAutoChooser("Auto1");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public AutoGenerator( Elevator m_elevator, Claw m_claw){
        elevator = m_elevator;
        claw = m_claw;


       InstantCommand command1 = new InstantCommand(                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        () ->m_elevator.toPosition(m_elevator.positionCoral1));  

        Command command2 = new IntakeCoral(elevator,claw);
        
        InstantCommand command3 = new InstantCommand(
            () ->m_claw.toPosition(m_claw.positionCoral1));  
            
        Command auto1 = AutoCommand1();
   
        defineCommands(command1, command2, command3);
        
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
