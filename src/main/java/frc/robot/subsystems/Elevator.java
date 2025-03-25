package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase{
    public final TalonFX elevatorRight = new TalonFX(22); 
    final TalonFX elevatorFollower = new TalonFX(21);
    public final DigitalInput limitLower = new DigitalInput(0);
    public final DigitalInput limitUpper = new DigitalInput(3);

    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private MotionMagicVoltage  magicToPos= new MotionMagicVoltage(0);
    private VoltageOut voltOutUp = new VoltageOut(.75);
    private VoltageOut voltOutDown = new VoltageOut(-.6);
    
    public double 
            positionAlgae1=13,positionAlgae2 = 24, 
            positionNet=43, positionProcessor=0, 
            positionCoral1=10.5, positionCoral2=7.0,
            positionCoral3=19.5,positionCoral4=42.25,
            positionIntake=0;
    public double elevatorPos=0;
    public static double setPos=0;
    boolean zeroed=false,atLowerLimit=false,atUpperLimit=false;

    private VoltageOut m_translationCharacterization = new VoltageOut(0);
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(0.2).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(3),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIDTranslation_State_Elev", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                elevatorRight.setControl(m_translationCharacterization.withOutput(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Translation_Rate_Elev", output.in(Volts));
            },
            null,
            this
        )
    );


public Elevator(){

    elevatorFollower.setControl(new Follower(22,true));
/* 
    SmartDashboard.putNumber("Elevator positionAlgae1", positionAlgae1);
    SmartDashboard.putNumber("Elevator positionAlgae2", positionAlgae2);
    SmartDashboard.putNumber("Elevator positionNet", positionNet);
    SmartDashboard.putNumber("Elevator positionProcessor", positionProcessor);
    SmartDashboard.putNumber("Elevator positionIntake", positionIntake);
    SmartDashboard.putNumber("Elevator positionCoral1", positionCoral1);
    SmartDashboard.putNumber("Elevator positionCoral2", positionCoral2);
    SmartDashboard.putNumber("Elevator positionCoral3", positionCoral3);
    SmartDashboard.putNumber("Elevator positionCoral4", positionCoral4);
*/
    
    configure();
    stopElevator();
    elevatorRight.setPosition(0);
    elevatorFollower.setPosition(0);

  

}

    public void periodic(){
//        if (!Robot.enabled) zeroed=false;
//        if(!zeroed && Robot.enabled)zeroElevator();

    //atLowerLimit=!limitLower.get()&& elevatorRight.getVelocity().getValueAsDouble()<0.01 && 
    //                elevatorRight.getStatorCurrent().getValueAsDouble()>1;
//        atLowerLimit=!limitLower.get();
//        atUpperLimit=!limitUpper.get();

  //      double vel=elevatorRight.getVelocity().getValueAsDouble();
        elevatorPos=elevatorRight.getPosition().getValueAsDouble();
        
        if(setPos>1 && Math.abs(elevatorPos-setPos)<.5 ) LED.elevatorAtPos=true;
        else LED.elevatorAtPos=false;

        SmartDashboard.putNumber("Elevator Pos", elevatorPos);


//        SmartDashboard.putBoolean("Elevator LLS",atLowerLimit);
//        SmartDashboard.putBoolean("Elevator ULS",atUpperLimit);
        // check mag limit switches
//        if (atUpperLimit && vel>0 ) stopElevator();
//        if (atLowerLimit && vel<0 ) stopElevator();
        
//        SmartDashboard.putNumber("Elevator set", elevatorRight.getClosedLoopReference().getValueAsDouble());
    }


    public Command ManualUp() {
        return runOnce( () -> {manualUp();});
        }

    public void manualUp(){
        if (!atUpperLimit)  elevatorRight.setControl(voltOutUp);
        else stopElevator();
    }


    public Command ManualDown() {
        return runOnce( () -> {manualDown();});
        }

    public void manualDown(){
        elevatorRight.setControl(voltOutDown);
    }


    public Command RampUp() {
        return runOnce( () -> {rampUp();});
    }

    public void rampUp(){
        double v = elevatorRight.getMotorVoltage().getValueAsDouble();
        v=v+.1;        
        if (!atUpperLimit) elevatorRight.setControl(new VoltageOut(v));
        else stopElevator();
        SmartDashboard.putNumber("Elevator PeakVolt", v);
    }

    public Command RampDown() {
        return runOnce( () -> {rampDown();});
    }
    
    public void rampDown(){
        double v = elevatorRight.getMotorVoltage().getValueAsDouble();
        v=v-.1;
        if (!atLowerLimit) elevatorRight.setControl(new VoltageOut(v));
        else stopElevator();
        SmartDashboard.putNumber("Elevator PeakVolt", v);
    }


    public Command ToPosition(double pos) {
        return runOnce( () -> {toPosition(pos);});
        }

    public void toPosition(double pos){
        setPos=pos;
        elevatorRight.setControl(magicToPos.withPosition(pos));
    }


    public void toPositionC1(){
        elevatorRight.setControl(magicToPos.withPosition(positionCoral1));
    }

    public void toPositionC2(){
        elevatorRight.setControl(magicToPos.withPosition(positionCoral2));
    }

    public void toPositionC3(){
        elevatorRight.setControl(magicToPos.withPosition(positionCoral3));
    }

    public void toPositionC4(){
        elevatorRight.setControl(magicToPos.withPosition(positionCoral4));
    }

    public void toPositionA1(){
        elevatorRight.setControl(magicToPos.withPosition(positionAlgae1));
    }

    public void toPositionA2(){
        elevatorRight.setControl(magicToPos.withPosition(positionAlgae2));
    }

    public void toPositionN(){
        elevatorRight.setControl(magicToPos.withPosition(positionNet));
    }

    public Command StopElevator() {
        return runOnce( () -> {stopElevator();});
        }

    public void stopElevator(){
        elevatorRight.stopMotor();
    }


 
    public void zeroElevator(){
        if(!limitLower.get()){
            while(!limitLower.get()){
                manualUp();
            }
            stopElevator();
        }

        manualDown();
        
        int i = 0;
        while(elevatorRight.getVelocity().getValueAsDouble()<-0.001 && i>20){
            i++;
        }
        stopElevator();
        zeroed=true;
        elevatorRight.setPosition(0);
        elevatorFollower.setPosition(0);
        elevatorPos=elevatorRight.getPosition().getValueAsDouble();
    }




  
  public void configure(){

    cfg.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;

    cfg.MotionMagic.MotionMagicCruiseVelocity=60;//60
    cfg.MotionMagic.MotionMagicAcceleration=80;//80
    cfg.MotionMagic.MotionMagicJerk=160;   //120

    cfg.Slot0.GravityType=GravityTypeValue.Elevator_Static;

    cfg.Slot0.kG=0.495;//0.495
    cfg.Slot0.kP=2;//  .120898
    cfg.Slot0.kI=0;
    cfg.Slot0.kD=0;
    cfg.Slot0.kS=0.0;//0
    cfg.Slot0.kV=.4;//.1239
    cfg.Slot0.kA=0;

    cfg.CurrentLimits.StatorCurrentLimit=80;
    cfg.CurrentLimits.StatorCurrentLimitEnable=true;
    cfg.CurrentLimits.SupplyCurrentLimit=50;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable=true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold=44;

    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable=true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold=0;


    elevatorRight.getConfigurator().apply(cfg);

  }

  public void updateConstants(){
    /* 
    positionAlgae1= SmartDashboard.getNumber("Elevator positionAlgae1", positionAlgae1);
    positionAlgae2= SmartDashboard.getNumber("Elevator positionAlgae2", positionAlgae2);
    positionNet= SmartDashboard.getNumber("Elevator positionNet", positionNet);
    positionProcessor= SmartDashboard.getNumber("Elevator positionProcessor", positionProcessor);
    positionIntake= SmartDashboard.getNumber("Elevator positionIntake", positionIntake);
    positionCoral1= SmartDashboard.getNumber("Elevator positionCoral1", positionCoral1);
    positionCoral2= SmartDashboard.getNumber("Elevator positionCoral2", positionCoral2);
    positionCoral3= SmartDashboard.getNumber("Elevator positionCoral3", positionCoral3);
    positionCoral4= SmartDashboard.getNumber("Elevator positionCoral4", positionCoral4);
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