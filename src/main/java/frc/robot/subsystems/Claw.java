package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

private TalonFX clawMotor = new TalonFX(11);
private TalonFX frontRollerMotor = new TalonFX(12);
private TalonFXConfiguration cfg = new TalonFXConfiguration();
private CANcoderConfiguration cfgEnc = new CANcoderConfiguration();
private CANcoder clawEncoder = new CANcoder(21);

private MotionMagicTorqueCurrentFOC magicToPos= new MotionMagicTorqueCurrentFOC(0);
private VoltageOut voltOutUp = new VoltageOut(6);
private VoltageOut voltOutDown = new VoltageOut(-3);
    
public double pos1=0,pos2=.5,pos3=.75,pos4=1.0,pos5=1.2;


private DigitalInput limitLower = new DigitalInput(6);
private DigitalInput limitUpper = new DigitalInput(5);
private DigitalInput coralSensor = new DigitalInput(7);

private boolean atLowerLimit=false,atUpperLimit=false;
private double currentLimit=30;
public Claw(){
    cfg.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;
    
    cfg.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID=21;
    cfg.Feedback.FeedbackRotorOffset=0;

    cfg.MotionMagic.MotionMagicCruiseVelocity=.8;
    cfg.MotionMagic.MotionMagicAcceleration=3;
    cfg.MotionMagic.MotionMagicJerk=50;   

    cfg.Slot0.GravityType=GravityTypeValue.Arm_Cosine;
    cfg.Slot0.kG=0;
    cfg.Slot0.kP=20;
    cfg.Slot0.kD=0;
    cfg.Slot0.kS=0;
    cfg.Slot0.kV=0;
    cfg.Slot0.kI=0;
    cfg.Slot0.kA=0;

    cfg.CurrentLimits.StatorCurrentLimit=120;
    cfg.CurrentLimits.StatorCurrentLimitEnable=true;
    cfg.CurrentLimits.SupplyCurrentLimit=40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent=300;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent=-300;

    clawMotor.getConfigurator().apply(cfg);

}


    public void periodic(){
        double v,i,encPos;
        v=clawMotor.getMotorVoltage().getValueAsDouble();
        i=clawMotor.getStatorCurrent().getValueAsDouble();
        encPos=clawEncoder.getPosition().getValueAsDouble();
        atLowerLimit=limitLower.get();
        atUpperLimit=limitUpper.get();

        SmartDashboard.putNumber("Claw Motor Pos", 
            clawMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elev Volt",v);
        SmartDashboard.putNumber("Elev Current", i);

        // check mag limit switches
        if (atUpperLimit && v>0 ) stopClaw();
        if (atLowerLimit && v<0 ) stopClaw();
        if (i>currentLimit) stopClaw();

    }


    public Command manualUpCommand() {
        return runOnce( () -> {manualUp();});
    }

    public void manualUp(){
        if (!atUpperLimit)  clawMotor.setControl(voltOutUp);
        else stopClaw();
    }


    public Command manualDownCommand() {
        return runOnce( () -> {manualDown();});
    }

    public void manualDown(){
        if (!atLowerLimit) clawMotor.setControl(voltOutDown);
        else stopClaw();
    }


    public Command toPositionCommand(double pos) {
        return runOnce( () -> {toPosition(pos);});
    }

    public void toPosition(double pos){
        clawMotor.setControl(magicToPos.withPosition(pos));
    }


    public Command stopClawCommand() {
        return runOnce( () -> {stopClaw();});
    }

    public void stopClaw(){
        clawMotor.stopMotor();
    }

}
