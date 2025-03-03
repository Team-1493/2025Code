package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
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

private TalonFX clawMotor = new TalonFX(23);
private TalonFX clawFrontRoller = new TalonFX(24);
private TalonFX clawRearRoller = new TalonFX(25);
private TalonFXConfiguration cfgRollers = new TalonFXConfiguration();
private TalonFXConfiguration cfg = new TalonFXConfiguration();
private CANcoderConfiguration cfgEnc = new CANcoderConfiguration();
private CANcoder clawEncoder = new CANcoder(27);

private MotionMagicVoltage magicToPos= new MotionMagicVoltage(0);
private PositionVoltage positionVolt = new PositionVoltage(0);
private VoltageOut voltOutUp = new VoltageOut(1);
private VoltageOut voltOutDown = new VoltageOut(-.5);
private VoltageOut  voltOutRearForward= new VoltageOut(4);
private VoltageOut  voltOutRearReverse= new VoltageOut(-3);
private VoltageOut  voltOutFrontForward= new VoltageOut(4);
private VoltageOut  voltOutFrontReverse= new VoltageOut(-4);
private VoltageOut  voltOutSpitCoral = new VoltageOut(-8);
private VoltageOut  voltOutFrontHold= new VoltageOut(-.6);
private VoltageOut  voltOutRearHold= new VoltageOut(.6);
private VoltageOut  voltOutFrontSpitAlgae= new VoltageOut(12);
private VoltageOut  voltOutRearSpitAlgae= new VoltageOut(-12);
private VoltageOut  voltOutFrontRevSpitAlgae= new VoltageOut(-12);
private VoltageOut  voltOutRearRevSpitAlgae= new VoltageOut(12);

    

public double 
        positionAlgae1=-.125,positionAlgae2 = -0.125, 
        positionNet=.13,positionProcessor=-0.17, 
        positionCoral1=.25, positionCoral2=.25,
        positionCoral3=.25,positionCoral4=.205,
        positionIntake=0.322,positionNeutral=0.25;   


private DigitalInput limitLower = new DigitalInput(4);
private DigitalInput limitUpper = new DigitalInput(1);
private DigitalInput coralSensor = new DigitalInput(7);

private boolean atLowerLimit=false,atUpperLimit=false;
public boolean hasCoral = false, prevHasCoral=false;
public boolean hasAlgae=false;
int coralCounter=0,algaeCounter=0;
private double voltage,current;
public double encPosition,rearRollerCurrent;

public Claw(){
    SmartDashboard.putNumber("Claw Pos Algae1", positionAlgae1);
    SmartDashboard.putNumber("Claw Pos Algae2", positionAlgae2);
    SmartDashboard.putNumber("Claw Pos Net", positionNet);
    SmartDashboard.putNumber("Claw Pos Processor", positionProcessor);
    SmartDashboard.putNumber("Claw Pos Coral1", positionCoral1);
    SmartDashboard.putNumber("Claw Pos Coral2", positionCoral2);
    SmartDashboard.putNumber("Claw Pos Coral3", positionCoral3);
    SmartDashboard.putNumber("Claw Pos Coral4", positionCoral4);
    SmartDashboard.putNumber("Claw Pos Intake", positionIntake);
    SmartDashboard.putNumber("Claw Pos Neutral", positionNeutral);

    configure();
}

    public void periodic(){
        voltage=clawMotor.getMotorVoltage().getValueAsDouble();

        current=clawMotor.getStatorCurrent().getValueAsDouble();
        double velocity=clawEncoder.getVelocity().getValueAsDouble();
        encPosition=clawEncoder.getAbsolutePosition().getValueAsDouble();

        atUpperLimit=false;
        // check mag limit switch
        if (atUpperLimit && velocity>0 ) stopClaw();
        if (atLowerLimit && voltage<0 ) stopClaw();
//        if (current>currentLimit) stopClaw();

        checkForCoral();
        checkForAlgae();
      

        if (prevHasCoral &&  !hasCoral) stopRollers();

        SmartDashboard.putNumber("Claw Enc AbsPos", encPosition);    
        SmartDashboard.putNumber("Claw Enc SetPosition", clawMotor.getClosedLoopReference().getValueAsDouble());    

        SmartDashboard.putNumber("Claw Voltqqq",voltage);
        SmartDashboard.putNumber("Claw Current", current);

        SmartDashboard.putBoolean("Claw LLS",atLowerLimit);
        SmartDashboard.putBoolean("Claw ULS",atUpperLimit);

        SmartDashboard.putBoolean("Coral Sensor", hasCoral);
        SmartDashboard.putBoolean("Algae Sensor", hasAlgae);
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
        return runOnce( () -> {toPosition(pos);
        SmartDashboard.putNumber("toPos", pos);});
    }

    public void toPosition(double pos){
        clawMotor.setControl(magicToPos.withPosition(pos));
//        clawMotor.setControl(positionVolt.withPosition(pos));
    }


    public void toPositionC1(){
        clawMotor.setControl(magicToPos.withPosition(positionCoral1));
    }

    public void toPositionC2(){
        clawMotor.setControl(magicToPos.withPosition(positionCoral2));
    }

    public void toPositionC3(){
        clawMotor.setControl(magicToPos.withPosition(positionCoral3));
    }

    public void toPositionC4(){
        clawMotor.setControl(magicToPos.withPosition(positionCoral4));
    }

    public void toPositionA1(){
        clawMotor.setControl(magicToPos.withPosition(positionAlgae1));
    }

    public void toPositionA2(){
        clawMotor.setControl(magicToPos.withPosition(positionAlgae2));
    }

    public void toPositionN(){
        clawMotor.setControl(magicToPos.withPosition(positionNet));
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

    public Command SpitCoral() {
        return runOnce( () -> {spitCoral();});
    }

    public void spitCoral(){
        clawRearRoller.setControl(voltOutSpitCoral);
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

    public void holdAlgae(){
        rollersRun(-1,1);
    }


    public Command SpitAlgae() {
        return runOnce( () -> {spitAlgae();});
    }

    public void spitAlgae(){
        clawFrontRoller.setControl(voltOutFrontSpitAlgae);
        clawRearRoller.setControl(voltOutRearSpitAlgae);
        hasAlgae=false;
    }

    public void spitReverseAlgae(){
        clawFrontRoller.setControl(voltOutFrontRevSpitAlgae);
        clawRearRoller.setControl(voltOutRearRevSpitAlgae);
        hasAlgae=false;
    }

    public void rollersRun(double front,double rear){
        clawFrontRoller.setControl(new VoltageOut(front));
        clawRearRoller.setControl(new VoltageOut(rear));
    }

    public void frontRollerStop(){
        clawFrontRoller.stopMotor();
    }
    
    public void stopRollers(){
        clawFrontRoller.stopMotor();
        clawRearRoller.stopMotor();
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


    // Check if we have Coral
    private void checkForCoral(){
            prevHasCoral=hasCoral;
            if (!coralSensor.get()) hasCoral=true;
            else hasCoral=false;
    }

    public boolean pickedUpCoral(){
        return (!prevHasCoral && hasCoral);
    }

    public boolean shotCoral(){
        return (prevHasCoral && !hasCoral);
    }

    private void checkForAlgae(){

        if( (Math.abs(clawRearRoller.getStatorCurrent().getValueAsDouble())>3)
        && Math.abs(clawRearRoller.getVelocity().getValueAsDouble())<0.1){
            algaeCounter ++;
            if (algaeCounter>3) hasAlgae=true;
            }

        else {
            hasAlgae=false;
            algaeCounter=0;
        } 
    }


    // Configure claw motor
    public void configure(){
    cfg.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;

    cfg.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID=27;
    
    cfg.Slot0.GravityType=GravityTypeValue.Arm_Cosine;
    cfg.Slot0.kG = 0.31;
    cfg.Slot0.kP = 9;
    cfg.Slot0.kI = 0;
    cfg.Slot0.kD = 0;
    cfg.Slot0.kS = 0.07;
    cfg.Slot0.kV = 5.25;
    cfg.Slot0.kA = 0.02;
 
//    cfg.MotorOutput.PeakForwardDutyCycle=.5;
//    cfg.MotorOutput.PeakForwardDutyCycle=-.5;

    cfg.MotionMagic.MotionMagicCruiseVelocity = 0.6;
    cfg.MotionMagic.MotionMagicAcceleration = 1.5;
    cfg.MotionMagic.MotionMagicJerk=30;   

    cfg.CurrentLimits.StatorCurrentLimit = 120;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 120;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;
    
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable=true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold=.332;

    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable=true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold=-0.26;

    clawMotor.getConfigurator().apply(cfg);

    // Configure Roller Motors
    cfgRollers.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
    cfgRollers.MotorOutput.NeutralMode=NeutralModeValue.Brake;
    cfgRollers.CurrentLimits.StatorCurrentLimit=80;
    cfgRollers.CurrentLimits.StatorCurrentLimitEnable=true;
    cfgRollers.CurrentLimits.SupplyCurrentLimit=50;
    cfgRollers.CurrentLimits.SupplyCurrentLimitEnable=true;
    clawFrontRoller.getConfigurator().apply(cfgRollers);
    clawRearRoller.getConfigurator().apply(cfgRollers);


    // Cponfigure encoder
    cfgEnc.MagnetSensor.AbsoluteSensorDiscontinuityPoint=0.7;
    cfgEnc.MagnetSensor.MagnetOffset=-0.67553;
    cfgEnc.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;
    
    clawEncoder.getConfigurator().apply(cfgEnc);
    }


    public void updateConstants(){
        positionAlgae1 = SmartDashboard.getNumber("Claw Pos Algae1", positionAlgae1);
        positionAlgae2 = SmartDashboard.getNumber("Claw Pos Algae2", positionAlgae2);
        positionNet = SmartDashboard.getNumber("Claw Pos Net", positionNet);
        positionProcessor = SmartDashboard.getNumber("Claw Pos Processor", positionProcessor);
        positionCoral1 = SmartDashboard.getNumber("Claw Pos Coral1", positionCoral1);
        positionCoral2 = SmartDashboard.getNumber("Claw Pos Coral2", positionCoral2);
        positionCoral3 = SmartDashboard.getNumber("Claw Pos Coral3", positionCoral3);
        positionCoral4 = SmartDashboard.getNumber("Claw Pos Coral4", positionCoral4);
        positionIntake = SmartDashboard.getNumber("Claw Pos Intake", positionIntake);
        positionNeutral = SmartDashboard.getNumber("Claw Pos Neutral", positionNeutral);
        SmartDashboard.putNumber("Claw pN", positionNeutral);    
    }

}
