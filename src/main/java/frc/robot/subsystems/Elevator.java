package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    final TalonFX elevatorRight = new TalonFX(9); 
    final Follower elevatorLeft = new Follower(10,true );
    final DigitalInput limitLower = new DigitalInput(8);
    final DigitalInput limitUpper = new DigitalInput(9);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    CANcoder encoder = new CANcoder(21);
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    MotionMagicTorqueCurrentFOC magic= new MotionMagicTorqueCurrentFOC(null);
    VoltageOut voltOutUp = new VoltageOut(.2);
    VoltageOut voltOutDown = new VoltageOut(.1);


public Elevator(){

    // Applyy encoder configurations
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint=1;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = 0.4;//  Set to correct value
    encoder.getConfigurator().apply(cc_cfg);


    // Apply motor configurations

    cfg.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;

    cfg.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID=encoder.getDeviceID();
    cfg.Feedback.SensorToMechanismRatio=1;
   // cfg.Feedback.RotorToSensorRatio=#;  set this value for synching or fusing to remote encoder

    cfg.MotionMagic.MotionMagicCruiseVelocity=2;
    cfg.MotionMagic.MotionMagicAcceleration=8;
    cfg.MotionMagic.MotionMagicJerk=100;   

    cfg.Slot0.kG=1;
    cfg.Slot0.kP=3;
    cfg.Slot0.kD=0;
    cfg.Slot0.kS=0;
    cfg.Slot0.kV=0;
    cfg.Slot0.kI=0;
    cfg.Slot0.kA=0;

    cfg.CurrentLimits.StatorCurrentLimit=100;
    cfg.CurrentLimits.StatorCurrentLimitEnable=true;
    cfg.CurrentLimits.SupplyCurrentLimit=40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent=100;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent=-100;

    //  TODO  - add remote limit switch using encoder

    elevatorRight.getConfigurator().apply(cfg);


}

    public void periodic(){
        SmartDashboard.putNumber("Elev Pos-Rem", elevatorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elev Pos-Enc", encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elev PosAbs-Enc", encoder.getAbsolutePosition().getValueAsDouble());

        // check mag limit switches
        if (limitUpper.get() && elevatorRight.getMotorVoltage().getValueAsDouble()>0 ) stopElevator();
        if (limitLower.get() && elevatorRight.getMotorVoltage().getValueAsDouble()<0 ) stopElevator();
    }


    public Command manualUpCommand() {
        return runOnce( () -> {manualUp();});
        }

    public void manualUp(){
        if (!limitUpper.get())  elevatorRight.setControl(voltOutUp);
        else stopElevator();
    }


    public Command manualDownCommand() {
        return runOnce( () -> {manualUp();});
        }

    public void manualDown(){
        if (!limitLower.get()) elevatorRight.setControl(voltOutUp);
        else stopElevator();
    }

    public Command stopElevatorCommand() {
        return runOnce( () -> {manualUp();});
        }

    public void stopElevator(){
        elevatorRight.stopMotor();
    }



}