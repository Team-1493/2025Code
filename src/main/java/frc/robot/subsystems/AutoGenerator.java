package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoGenerator {
    public SendableChooser<Command> autoChooser;

    private void defineCommands(Command command1, Command command2, Command command3){
        NamedCommands.registerCommand("testAction", command1);
        NamedCommands.registerCommand("testAction", command2);
        NamedCommands.registerCommand("testAction", command3);
    }

    private void autoChooserInit(){
        autoChooser = AutoBuilder.buildAutoChooser("testAuto1");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public AutoGenerator(Command command1, Command command2, Command command3){
        defineCommands(command1, command2, command3);
        autoChooserInit();
    }


}
