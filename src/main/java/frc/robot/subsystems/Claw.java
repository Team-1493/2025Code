package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Claw extends SubsystemBase {

private TalonFX clawMotor = new TalonFX(23);
private TalonFX clawFrontRoller = new TalonFX(24);
private TalonFX clawRearRoller = new TalonFX(25);
private TalonFX chuteRoller = new TalonFX(28);

private TalonFXConfiguration cfgRollers = new TalonFXConfiguration();
private TalonFXConfiguration cfg = new TalonFXConfiguration();
private CANcoderConfiguration cfgEnc = new CANcoderConfiguration();
private CANcoder clawEncoder = new CANcoder(27);

private MotionMagicVoltage magicToPos= new MotionMagicVoltage(0);
private VoltageOut voltOutUp = new VoltageOut(1);
private VoltageOut voltOutChuteRoller = new VoltageOut(-5); // -3.8
private VoltageOut voltOutChuteRolleReverse = new VoltageOut(5); // -3.8

private VoltageOut voltOutDown = new VoltageOut(-.5);
private VoltageOut  voltOutRearForward= new VoltageOut(4);
private VoltageOut  voltOutRearReverse= new VoltageOut(-3.25);
private VoltageOut  voltOutFrontForward= new VoltageOut(4);
private VoltageOut  voltOutFrontReverse= new VoltageOut(-4);
private VoltageOut  voltOutSpitCoral = new VoltageOut(-8);
private VoltageOut  voltOutSpitCoralL1 = new VoltageOut(-1.5);
private DutyCycleOut dutyFrontSpitAlgae = new DutyCycleOut(3);
private DutyCycleOut dutyRearSpitAlgae = new DutyCycleOut(-3); 
private VoltageOut  voltOutFrontRevSpitAlgae= new VoltageOut(-12);
private VoltageOut  voltOutRearRevSpitAlgae= new VoltageOut(12);
private VoltageOut m_rotationCharacterization = new VoltageOut(0);
public static boolean spittingNet =  false;

    
// net 0.1
public double 
        positionAlgae1=-0.035,positionAlgae2 = -0.035, 
        positionNet = 0.05,positionProcessor=-0.1, 
        positionCoral1= 0.12, positionCoral2= 0.314,
        positionCoral3 = 0.314,positionCoral4= 0.235,
        positionIntake = 0.372, positionNeutral = 0.306;//0.306; (.15)   

//private DigitalInput limitLower = new DigitalInput(1);
private DigitalInput limitUpper = new DigitalInput(4);
public DigitalInput coralSensor = new DigitalInput(6);

private boolean atLowerLimit=false,atUpperLimit=false;
public boolean hasCoral = false, prevHasCoral=false;
public boolean hasAlgae=false;
int coralCounter=0,algaeCounter=0;
public double encPosition,rearRollerCurrent;


   private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(0.1).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(1),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                clawMotor.setControl(m_rotationCharacterization.withOutput(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );



public Claw(){
/*     SmartDashboard.putNumber("Claw Pos Algae1", positionAlgae1);
    SmartDashboard.putNumber("Claw Pos Algae2", positionAlgae2);
    SmartDashboard.putNumber("Claw Pos Net", positionNet);
    SmartDashboard.putNumber("Claw Pos Processor", positionProcessor);
    SmartDashboard.putNumber("Claw Pos Coral1", positionCoral1);
    SmartDashboard.putNumber("Claw Pos Coral2", positionCoral2);
    SmartDashboard.putNumber("Claw Pos Coral3", positionCoral3);
    SmartDashboard.putNumber("Claw Pos Coral4", positionCoral4);
    SmartDashboard.putNumber("Claw Pos Intake", positionIntake);
    SmartDashboard.putNumber("Claw Pos Neutral", positionNeutral);
*/
    SmartDashboard.putNumber("Claw Pos Net", positionNet);

    configure();




    
}

    public void periodic(){
        encPosition=clawEncoder.getAbsolutePosition().getValueAsDouble();

        atUpperLimit=!limitUpper.get();
//        if (atUpperLimit && clawEncoder.getVelocity().getValueAsDouble()>0) stopClaw(); 
        checkForCoral();
        checkForAlgae();
        

        if (prevHasCoral &&  !hasCoral) stopRollers();
        if(!prevHasCoral && hasCoral) stopRollers();

        SmartDashboard.putNumber("Claw Enc AbsPos", encPosition);    
//        SmartDashboard.putNumber("Claw Enc SetPosition", clawMotor.getClosedLoopReference().getValueAsDouble());    

//        SmartDashboard.putBoolean("Claw ULS",atUpperLimit);

//        SmartDashboard.putBoolean("Coral Sensor", hasCoral);
//        SmartDashboard.putBoolean("Algae Sensor", hasAlgae);
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

    public Command SpitCoralL1() {
        return runOnce( () -> {spitCoralL1();});
    }

    public void spitCoralL1(){
        clawRearRoller.setControl(voltOutSpitCoralL1);
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
        rollersRun(-1.25,1.25);
    }


    public Command SpitAlgae() {
        return runOnce( () -> {spitAlgae();});
    }

    public void spitAlgae(){
        clawFrontRoller.setControl(dutyFrontSpitAlgae);
      clawRearRoller.setControl(dutyRearSpitAlgae);
  //      clawFrontRoller.setControl(torqueFrontSpitAlgae);
//        clawRearRoller.setControl(torqueRearSpitAlgae);
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

    public void chuteRollerStop(){
        chuteRoller.stopMotor();
    }

    public void chuteRollerRun(){
        chuteRoller.setControl(voltOutChuteRoller);
    }

    public void chuteRollerRunReverse(){
        chuteRoller.setControl(voltOutChuteRolleReverse);
    }
    

    public void stopRollers(){
        clawFrontRoller.stopMotor();
        clawRearRoller.stopMotor();
        chuteRoller.stopMotor();
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
            if (!coralSensor.get()) {
                hasCoral=true;
                LED.hasCoral=true;
            }
            else {
                hasCoral=false;
                LED.hasCoral=false;
            }

       


    }

    public boolean pickedUpCoral(){
        return (!prevHasCoral && hasCoral);
    }

    public boolean shotCoral(){
        return (prevHasCoral && !hasCoral);
    }

    private void checkForAlgae(){

        if( (Math.abs(clawRearRoller.getStatorCurrent().getValueAsDouble())>80)
        && Math.abs(clawRearRoller.getVelocity().getValueAsDouble())<0.05){
            algaeCounter ++;
            if (algaeCounter>20) hasAlgae=true;
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
 
    cfg.MotionMagic.MotionMagicCruiseVelocity = 3 ; // 2;
    cfg.MotionMagic.MotionMagicAcceleration = 6;  //4
    cfg.MotionMagic.MotionMagicJerk=20; //30  

    cfg.CurrentLimits.StatorCurrentLimit = 80;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;
    
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable=true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.375;

    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable=true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.11;

//    cfg.HardwareLimitSwitch.ForwardLimitEnable = true;
//    cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
//    cfg.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
//    cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;


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
    cfgEnc.MagnetSensor.AbsoluteSensorDiscontinuityPoint=0.8;
    cfgEnc.MagnetSensor.MagnetOffset = -0.473;
    cfgEnc.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;
    
    clawEncoder.getConfigurator().apply(cfgEnc);
    }


    public void updateConstants(){
        /* 

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
*/
    }



    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.dynamic(direction);
    }


}
