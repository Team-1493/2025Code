package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
private VoltageOut  voltOutFrontHold= new VoltageOut(-.3);
private VoltageOut  voltOutRearHold= new VoltageOut(.3);
private VoltageOut  voltOutFrontSpitAlgae= new VoltageOut(4);
private VoltageOut  voltOutRearSpitAlgae= new VoltageOut(-4);


    

public double 
        positionAlgae1=0,positionAlgae2 = 0, positionNet, 
        positionCoral1=0, positionCoral2=.1,
        positionCoral3=.15,positionCoral4=.2,
        positionIntake=0.365;   


private DigitalInput limitLower = new DigitalInput(4);
private DigitalInput limitUpper = new DigitalInput(1);
private DigitalInput coralSensor = new DigitalInput(5);

private boolean atLowerLimit=false,atUpperLimit=false;
private double currentLimit=30;
public boolean hasCoral = false;
public boolean hasAlgae=false;
int coralCounter=0,algaeCounter=0;
private double voltage,current;
public double encPosition,rearRollerCurrent;

public Claw(){
    SmartDashboard.putNumber("Claw MMacc", 1.5);
    SmartDashboard.putNumber("Claw MMvel", .6);
    SmartDashboard.putNumber("Claw MMjerk", 30);

    SmartDashboard.putNumber("Claw kG", .305);
    SmartDashboard.putNumber("Claw kP", 2.5);
    SmartDashboard.putNumber("Claw kI", 0);
    SmartDashboard.putNumber("Claw kD", 0);
    SmartDashboard.putNumber("Claw kS", 0.07);
    SmartDashboard.putNumber("Claw kA", 0.5);

    SmartDashboard.putNumber("Claw StatorCL", 120);
    SmartDashboard.putNumber("Claw SupplyCL", 120);
    SmartDashboard.putNumber("Claw Pos Algae1", positionAlgae1);
    SmartDashboard.putNumber("Claw Pos Algae2", positionAlgae2);
    SmartDashboard.putNumber("Claw Pos Net", positionNet);
    SmartDashboard.putNumber("Claw Pos Coral1", positionCoral1);
    SmartDashboard.putNumber("Claw Pos Coral2", positionCoral2);
    SmartDashboard.putNumber("Claw Pos Coral3", positionCoral3);
    SmartDashboard.putNumber("Claw Pos Coral4", positionCoral4);
    SmartDashboard.putNumber("Claw Pos Intake", positionIntake);



    configure();

}


    public void periodic(){
        voltage=clawMotor.getMotorVoltage().getValueAsDouble();
        current=clawMotor.getStatorCurrent().getValueAsDouble();
        double velocity=clawEncoder.getVelocity().getValueAsDouble();
        encPosition=clawEncoder.getAbsolutePosition().getValueAsDouble();


//        atLowerLimit=!limitLower.get()|| encPosition<-.3;
//        atLowerLimit=encPosition<-.1;
 //       atUpperLimit=!limitUpper.get();
        atUpperLimit=false;
        // check mag limit switch
        if (atUpperLimit && velocity>0 ) stopClaw();
        if (atLowerLimit && voltage<0 ) stopClaw();
//        if (current>currentLimit) stopClaw();

        checkForCoral();
        checkForAlgae();

        SmartDashboard.putNumber("Claw Enc AbsPos", encPosition);    

        SmartDashboard.putNumber("Claw Voltqqq",voltage);
        SmartDashboard.putNumber("Claw Current", current);
        SmartDashboard.putNumber("Roller Current", clawRearRoller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Roller Vel", clawRearRoller.getVelocity().getValueAsDouble());

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

    public void holdAlgae(){
        clawFrontRoller.setControl(voltOutFrontHold);
        clawRearRoller.setControl(voltOutRearHold);
    }


    public Command SpitAlgae() {
        return runOnce( () -> {spitAlgae();});
    }

    public void spitAlgae(){
        clawFrontRoller.setControl(voltOutFrontSpitAlgae);
        clawRearRoller.setControl(voltOutRearSpitAlgae);
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
            if (!coralSensor.get()) hasCoral=true;
            else hasCoral=false;
    }

    private void checkForAlgae(){

        if( (Math.abs(clawRearRoller.getStatorCurrent().getValueAsDouble())>1)
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

    double clawkG = SmartDashboard.getNumber("Claw kG", 0);
    double clawkP = SmartDashboard.getNumber("Claw kP", 1);
    double clawkI = SmartDashboard.getNumber("Claw kI", 0);
    double clawkD = SmartDashboard.getNumber("Claw kD", 0);
    double clawkS = SmartDashboard.getNumber("Claw kS", 0);
    double clawkA = SmartDashboard.getNumber("Claw kA", 0);

    double clawMMacc = SmartDashboard.getNumber("Claw MMacc", 3);
    double clawMMvel = SmartDashboard.getNumber("Claw MMvel", 0.8);
    double clawMMjerk = SmartDashboard.getNumber("Claw MMjerk", 50);

    double clawStatorCL = SmartDashboard.getNumber("Claw StatorCL", 60);
    double clawSupplyCL = SmartDashboard.getNumber("Claw SupplyCL", 40);
    
    positionAlgae1 = SmartDashboard.getNumber("Claw Pos Algae1", positionAlgae1);
    positionAlgae2 = SmartDashboard.getNumber("Claw Pos Algae2", positionAlgae2);
    positionNet = SmartDashboard.getNumber("Claw Pos Net", positionNet);
    positionCoral1 = SmartDashboard.getNumber("Claw Pos Coral1", positionCoral1);
    positionCoral2 = SmartDashboard.getNumber("Claw Pos Coral2", positionCoral2);
    positionCoral3 = SmartDashboard.getNumber("Claw Pos Coral3", positionCoral3);
    positionCoral4 = SmartDashboard.getNumber("Claw Pos Coral4", positionCoral4);
    positionIntake = SmartDashboard.getNumber("Claw Pos Intake", positionIntake);


    cfg.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;

    cfg.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID=27;
    
    cfg.Slot0.GravityType=GravityTypeValue.Arm_Cosine;
    cfg.Slot0.kG = clawkG;
    cfg.Slot0.kP = clawkP;
    cfg.Slot0.kI = clawkI;
    cfg.Slot0.kD = clawkD;
    cfg.Slot0.kS=clawkS;
    cfg.Slot0.kV=5.25;
    cfg.Slot0.kA=clawkA;

//    cfg.MotorOutput.PeakForwardDutyCycle=.5;
//    cfg.MotorOutput.PeakForwardDutyCycle=-.5;

    cfg.MotionMagic.MotionMagicCruiseVelocity = clawMMvel;
    cfg.MotionMagic.MotionMagicAcceleration = clawMMacc;
    cfg.MotionMagic.MotionMagicJerk=clawMMjerk;   
        SmartDashboard.putNumber("clawpk", clawkP);

    cfg.CurrentLimits.StatorCurrentLimit=clawStatorCL;
    cfg.CurrentLimits.StatorCurrentLimitEnable=true;
    cfg.CurrentLimits.SupplyCurrentLimit=clawSupplyCL;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;
    
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable=true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold=.380;

    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable=true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold=-0.1;

    SmartDashboard.putNumber("clawpk", clawkP);
    SmartDashboard.putNumber("clawgk", clawkG);

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


    // top possible position: 0.00708
    // lowest possible position: -0.

    // Cponfigure encoder
    cfgEnc.MagnetSensor.AbsoluteSensorDiscontinuityPoint=0.6;
    cfgEnc.MagnetSensor.MagnetOffset=0.355;
    cfgEnc.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;
    
    clawEncoder.getConfigurator().apply(cfgEnc);


    }


}
