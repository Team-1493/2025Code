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

    
public double positionCoralIn=.2,positionCoralOut=.5, 
    positionAlgae=0.7,positionMid=0.45;
public double maxClawToLower=positionMid-.05;

private DigitalInput limitLower = new DigitalInput(6);
private DigitalInput limitUpper = new DigitalInput(5);
private DigitalInput coralSensor = new DigitalInput(7);

private boolean atLowerLimit=false,atUpperLimit=false;
private double currentLimit=30;
public boolean hasCoral = false;
private double voltage,current,motorPosition;
public double encPosition,rearRollerCurrent;

public Claw(){
    SmartDashboard.putNumber("Claw kG", 0);
    SmartDashboard.putNumber("Claw MMacc", 3);
    SmartDashboard.putNumber("Claw MMvel", 0.8);
    SmartDashboard.putNumber("Claw MMjerk", 50);
    SmartDashboard.putNumber("Claw kP", 20);
    SmartDashboard.putNumber("Claw kD", 0);
    SmartDashboard.putNumber("Claw StatorCL", 60);
    SmartDashboard.putNumber("Claw SupplyCL", 40);
    SmartDashboard.putNumber("Claw TorqueCL", 100);
    SmartDashboard.putNumber("Claw Pos CoralIn", positionCoralIn);
    SmartDashboard.putNumber("Claw Pos CoralOut", positionCoralOut);
    SmartDashboard.putNumber("Claw Pos Algae", positionAlgae );
    SmartDashboard.putNumber("Claw Pos Mid", positionMid );


    configureClaw();

}


    public void periodic(){
        voltage=clawMotor.getMotorVoltage().getValueAsDouble();
        current=clawMotor.getStatorCurrent().getValueAsDouble();
        encPosition=clawEncoder.getAbsolutePosition().getValueAsDouble();
        rearRollerCurrent=clawFrontRoller.getStatorCurrent().getValueAsDouble();

        atLowerLimit=limitLower.get();
        atUpperLimit=limitUpper.get();

        // check mag limit switches
        if (atUpperLimit && voltage>0 ) stopClaw();
        if (atLowerLimit && voltage<0 ) stopClaw();
        if (current>currentLimit) stopClaw();

        checkForCoral();

        SmartDashboard.putNumber("Claw Enc AbsPos", encPosition);    

        SmartDashboard.putNumber("Claw Volt",voltage);
        SmartDashboard.putNumber("Claw Current", current);
        SmartDashboard.putNumber("Roller Current", rearRollerCurrent);


        SmartDashboard.putBoolean("Coral Sensor", hasCoral);
    }


    public Command ClawUp() {
        return runOnce( () -> {clawUp();});
    }

    public void clawUp(){
        if (!atUpperLimit)  clawMotor.setControl(voltOutUp);
        else stopClaw();
    }


    public Command ClawDown() {
        return runOnce( () -> {clawDown();});
    }

    public void clawDown(){
        if (!atLowerLimit) clawMotor.setControl(voltOutDown);
        else stopClaw();
    }


    public Command ToPosition(double pos) {
        return runOnce( () -> {toPosition(pos);});
    }

    public void toPosition(double pos){
        clawMotor.setControl(magicToPos.withPosition(pos));
    }


    public Command StopClaw() {
        return runOnce( () -> {stopClaw();});
    }

    public void stopClaw(){
        clawMotor.stopMotor();
    }


    public Command RearRollerFor() {
        return runOnce( () -> {rearRollerFor();});
    }

    public void rearRollerFor(){
        clawRearRoller.setControl(voltOutRearForward);
    }

    public Command RearRollerRev() {
        return runOnce( () -> {rearRollerRev();});
    }

    public void rearRollerRev(){
        clawRearRoller.setControl(voltOutRearReverse);
    }


    public void stopRollerRear(){
        clawRearRoller.stopMotor();
    }

    public Command FrontRollerFor() {
        return runOnce( () -> {frontRollerFor();});
    }

    public void frontRollerFor(){
        clawFrontRoller.setControl(voltOutFrontForward);
    }

    public Command FrontRollerRev() {
        return runOnce( () -> {frontRollerRev();});
    }

    public void frontRollerRev(){
        clawFrontRoller.setControl(voltOutFrontReverse);
    }


    public Command StopRollers() {
        return runOnce( () -> {
            clawFrontRoller.stopMotor();
            clawRearRoller.stopMotor();});
    }

    
    public Command RollersForward() {
        return runOnce( () -> {rearRollerFor();frontRollerFor();});
    }

    public Command RollersReverse() {
        return runOnce( () -> {rearRollerRev();frontRollerRev();});
    }


    public Command configure() {
        return runOnce( () -> {configureClaw();});
    }


    // Check if we have Coral
    private void checkForCoral(){
//        hasCoral=coralSensor.get();
        if (clawRearRoller.getVelocity().getValueAsDouble()>1 ) {
            if(clawRearRoller.getStatorCurrent().getValueAsDouble()>4) hasCoral=true;
                else hasCoral=false;
        }         
    }


    // Configure claw motor
    public void configureClaw(){

    double clawkG = SmartDashboard.getNumber("Claw kG", 0);
    double clawkP = SmartDashboard.getNumber("Claw kP", 20);
    double clawkD = SmartDashboard.getNumber("Claw kD", 0);
    double clawMMacc = SmartDashboard.getNumber("Claw MMacc", 3);
    double clawMMvel = SmartDashboard.getNumber("Claw MMvel", 0.8);
    double clawMMjerk = SmartDashboard.getNumber("Claw MMjerk", 50);
    double clawStatorCL = SmartDashboard.getNumber("Claw StatorCL", 60);
    double clawSupplyCL = SmartDashboard.getNumber("Claw SupplyCL", 40);
    double clawTorqueCL = SmartDashboard.getNumber("Claw TorqueCL", 100);
    positionCoralIn= SmartDashboard.getNumber("Claw Pos CoralIn", positionCoralIn);
    positionCoralOut= SmartDashboard.getNumber("Claw Pos CoralOut", positionCoralOut);
    positionAlgae= SmartDashboard.getNumber("Claw Pos Algae", positionAlgae );
    positionMid= SmartDashboard.getNumber("Claw Pos Mid", positionMid );




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
    cfg.Slot0.kP = clawkP;
    cfg.Slot0.kD = clawkD;
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
