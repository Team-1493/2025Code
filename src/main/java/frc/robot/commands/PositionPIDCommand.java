package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PositionPIDCommand extends Command{
    
    public CommandSwerveDrivetrain sd;
    public final Pose2d goalPose;
    public static final PIDConstants kTranslationPID = new PIDConstants(1.0,0,0);
    public static final PIDConstants kRotationPID = new PIDConstants(1.0,0,0);
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
    public static final Distance kPositionTolerance = Centimeter.of(2.0);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(1);    
    public static final Time kEndTriggerDebounce = Seconds.of(0.1);
    public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
    public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);

    
    private PPHolonomicDriveController mDriveController = 
             new PPHolonomicDriveController(kTranslationPID, kRotationPID);
    private final Timer timer = new Timer();

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;


    private PositionPIDCommand(CommandSwerveDrivetrain m_sd, Pose2d m_goalPose) {
        sd = m_sd;
        goalPose = m_goalPose;

        endTrigger = new Trigger(() -> {
        Pose2d diff = sd.getPose().relativeTo(goalPose);

        var position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);

        var rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            kRotationTolerance.getRotations(), 
            0.0, 
            1.0
        );


        var speed =sd.getSpeed() < kSpeedTolerance.in(MetersPerSecond);

            // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
            
            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(CommandSwerveDrivetrain sd, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(sd, goalPose).withTimeout(timeout).finallyDo(() -> {
            sd.driveRobotCentric(new ChassisSpeeds(0,0,0));
        });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;


        sd.driveRobotCentric(
            mDriveController.calculateRobotRelativeSpeeds(
                sd.getPose(), goalState
            )
        );

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}
