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
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

private TalonFX clawMotor = new TalonFX(11);
private TalonFX clawFrontRoller = new TalonFX(12);
private TalonFX clawRearRoller = new TalonFX(13);
private TalonFXConfiguration cfgRollers = new TalonFXConfiguration();
private TalonFXConfiguration cfg = new TalonFXConfiguration();
private CANcoderConfiguration cfgEnc = new CANcoderConfiguration();
private CANcoder clawEncoder = new CANcoder(21);

private MotionMagicTorqueCurrentFOC magicToPos= new MotionMagicTorqueCurrentFOC(0);
private VoltageOut voltOutUp = new VoltageOut(6);
private VoltageOut voltOutDown = new VoltageOut(-3);
private VoltageOut  voltOutRearForward= new VoltageOut(4);
private VoltageOut  voltOutRearReverse= new VoltageOut(-4);
private VoltageOut  voltOutFrontForward= new VoltageOut(4);
private VoltageOut  voltOutFrontReverse= new VoltageOut(-4);

    
public double pos1=.2,pos2=.7;


private DigitalInput limitLower = new DigitalInput(6);
private DigitalInput limitUpper = new DigitalInput(5);
private DigitalInput coralSensor = new DigitalInput(7);

private boolean atLowerLimit=false,atUpperLimit=false;
private double currentLimit=30;
public Claw(){
    SmartDashboard.putNumber("Claw kG", 0);
    SmartDashboard.putNumber("Claw MMacc", 3);
    SmartDashboard.putNumber("Claw MMvel", 0.8);
    SmartDashboard.putNumber("Claw MMjerk", 50);
    SmartDashboard.putNumber("Claw kP", 20);
    SmartDashboard.putNumber("Claw StatorCL", 60);
    SmartDashboard.putNumber("Claw SupplyCL", 40);
    SmartDashboard.putNumber("Claw TorqueCL", 100);
    SmartDashboard.putNumber("Claw Pos1", pos1);
    SmartDashboard.putNumber("Claw Pos2", pos2);


    configureClaw();

}


    public void periodic(){
        double v,i,encPos,encPosAbs;
        boolean hasCoral=coralSensor.get();
        v=clawMotor.getMotorVoltage().getValueAsDouble();
        i=clawMotor.getStatorCurrent().getValueAsDouble();
        encPos=clawEncoder.getPosition().getValueAsDouble();
        encPosAbs=clawEncoder.getAbsolutePosition().getValueAsDouble();
        atLowerLimit=limitLower.get();
        atUpperLimit=limitUpper.get();

        SmartDashboard.putNumber("Claw Motor Pos", 
            clawMotor.getPosition().getValueAsDouble());
        
        SmartDashboard.putNumber("Claw Enc Pos", encPos);
        SmartDashboard.putNumber("Claw Enc AbsPos", encPosAbs);    

        SmartDashboard.putNumber("Claw Volt",v);
        SmartDashboard.putNumber("Claw Current", i);

        SmartDashboard.putBoolean("Coral Sensor", hasCoral);

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


    public Command manualRearRollerForCommand() {
        return runOnce( () -> {manualRearRollerFor();});
    }

    public void manualRearRollerFor(){
        clawRearRoller.setControl(voltOutRearForward);
    }

    public Command manualRearRollerRevCommand() {
        return runOnce( () -> {manualRearRollerRev();});
    }

    public void manualRearRollerRev(){
        clawRearRoller.setControl(voltOutRearReverse);
    }

    public Command stopRollerRearCommand() {
        return runOnce( () -> {stopRollerFront();});
    }

    public void stopRollerRear(){
        clawRearRoller.stopMotor();
    }

    public Command manualFrontRollerForCommand() {
        return runOnce( () -> {manualFrontRollerFor();});
    }

    public void manualFrontRollerFor(){
        clawFrontRoller.setControl(voltOutFrontForward);
    }

    public Command manualFrontRollerRevCommand() {
        return runOnce( () -> {manualFrontRollerRev();});
    }

    public void manualFrontRollerRev(){
        clawFrontRoller.setControl(voltOutFrontReverse);
    }

    public Command stopRollerFrontCommand() {
        return runOnce( () -> {stopRollerFront();});
    }

    public void stopRollerFront(){
        clawFrontRoller.stopMotor();
    }


    public Command configure() {
        return runOnce( () -> {configureClaw();});
    }


    // Configure claw motor
    public void configureClaw(){

    double clawkG = SmartDashboard.getNumber("Claw kG", 0);
    double clawKp = SmartDashboard.getNumber("Claw kP", 20);
    double clawMMacc = SmartDashboard.getNumber("Claw MMacc", 3);
    double clawMMvel = SmartDashboard.getNumber("Claw MMvel", 0.8);
    double clawMMjerk = SmartDashboard.getNumber("Claw MMjerk", 50);
    double clawStatorCL = SmartDashboard.getNumber("Claw StatorCL", 60);
    double clawSupplyCL = SmartDashboard.getNumber("Claw SupplyCL", 40);
    double clawTorqueCL = SmartDashboard.getNumber("Claw TorqueCL", 100);
    pos1= SmartDashboard.getNumber("Claw Pos1", pos1);
    pos2= SmartDashboard.getNumber("Claw Pos2", pos2);


    cfg.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;

    cfg.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID=21;
    cfg.Feedback.FeedbackRotorOffset=0;

    cfg.MotionMagic.MotionMagicCruiseVelocity = clawMMvel;
    cfg.MotionMagic.MotionMagicAcceleration = clawMMacc;
    cfg.MotionMagic.MotionMagicJerk=clawMMjerk;   

    cfg.Slot0.GravityType=GravityTypeValue.Arm_Cosine;
    cfg.Slot0.kG = clawkG;
    cfg.Slot0.kP = clawKp;
    cfg.Slot0.kD=0;
    cfg.Slot0.kS=0;
    cfg.Slot0.kV=0;
    cfg.Slot0.kI=0;
    cfg.Slot0.kA=0;

    cfg.CurrentLimits.StatorCurrentLimit=clawStatorCL;
    cfg.CurrentLimits.StatorCurrentLimitEnable=true;
    cfg.CurrentLimits.SupplyCurrentLimit=clawSupplyCL;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent=clawTorqueCL;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent=-clawTorqueCL;

    clawMotor.getConfigurator().apply(cfg);


    // Configure Roller Motors
    cfgRollers.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
    cfgRollers.MotorOutput.NeutralMode=NeutralModeValue.Brake;
    cfgRollers.CurrentLimits.StatorCurrentLimit=60;
    cfgRollers.CurrentLimits.StatorCurrentLimitEnable=true;
    cfgRollers.CurrentLimits.SupplyCurrentLimit=40;
    cfgRollers.CurrentLimits.SupplyCurrentLimitEnable=true;
    clawFrontRoller.getConfigurator().apply(cfgRollers);
    clawRearRoller.getConfigurator().apply(cfgRollers);


    // Cponfigure encoder
    cfgEnc.MagnetSensor.AbsoluteSensorDiscontinuityPoint=1;
    cfgEnc.MagnetSensor.MagnetOffset=0;
    cfgEnc.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;
    
    clawEncoder.getConfigurator().apply(cfgEnc);


    }


}
