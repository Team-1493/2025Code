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

public class ActionCommands {

    static private PathConstraints constraints;
    
    private CommandSwerveDrivetrain sd;
    
    

public  ActionCommands(CommandSwerveDrivetrain m_sd,Elevator elevator,Claw claw){

    sd=m_sd;

    constraints = new PathConstraints(
        1.5, 
        1.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(450));
                
        }


        
            
        


}



