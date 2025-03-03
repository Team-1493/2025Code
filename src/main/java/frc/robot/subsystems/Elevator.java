package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.ElevatorToReef;

public class Elevator extends SubsystemBase{
    public final TalonFX elevatorRight = new TalonFX(22); 
    final TalonFX elevatorFollower = new TalonFX(21);
    public final DigitalInput limitLower = new DigitalInput(0);
    public final DigitalInput limitUpper = new DigitalInput(3);

    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private MotionMagicVoltage  magicToPos= new MotionMagicVoltage(0);
    private VoltageOut voltOutUp = new VoltageOut(.75);
    private VoltageOut voltOutDown = new VoltageOut(-.4);
    
    public double 
            positionAlgae1=15,positionAlgae2 = 30, 
            positionNet=43, positionProcessor=5, 
            positionCoral1=5.5, positionCoral2=7.5,
            positionCoral3=19.5,positionCoral4=42,
            positionIntake=0;
    public double elevatorPos=0;
    boolean zeroed=false,atLowerLimit=false,atUpperLimit=false;

public Elevator(){

    elevatorFollower.setControl(new Follower(22,true));
    SmartDashboard.putNumber("Elevator positionAlgae1", positionAlgae1);
    SmartDashboard.putNumber("Elevator positionAlgae2", positionAlgae2);
    SmartDashboard.putNumber("Elevator positionNet", positionNet);
    SmartDashboard.putNumber("Elevator positionProcessor", positionProcessor);
    SmartDashboard.putNumber("Elevator positionIntake", positionIntake);
    SmartDashboard.putNumber("Elevator positionCoral1", positionCoral1);
    SmartDashboard.putNumber("Elevator positionCoral2", positionCoral2);
    SmartDashboard.putNumber("Elevator positionCoral3", positionCoral3);
    SmartDashboard.putNumber("Elevator positionCoral4", positionCoral4);

    
    configure();
    stopElevator();
  

}

    public void periodic(){
//        if (!Robot.enabled) zeroed=false;
//        if(!zeroed && Robot.enabled)zeroElevator();

    //atLowerLimit=!limitLower.get()&& elevatorRight.getVelocity().getValueAsDouble()<0.01 && 
    //                elevatorRight.getStatorCurrent().getValueAsDouble()>1;
        atLowerLimit=!limitLower.get();
        atUpperLimit=!limitUpper.get();

        double v,i;
        v=elevatorRight.getMotorVoltage().getValueAsDouble();
        i=elevatorRight.getStatorCurrent().getValueAsDouble();
        double vel=elevatorRight.getVelocity().getValueAsDouble();
        elevatorPos=elevatorRight.getPosition().getValueAsDouble();
        
        SmartDashboard.putNumber("Elevator Pos", elevatorPos);
        SmartDashboard.putNumber("Elevator Volt",vel);
        SmartDashboard.putNumber("Elevator Current", i);


        SmartDashboard.putBoolean("Elevator LLS",atLowerLimit);
        SmartDashboard.putBoolean("Elevator ULS",atUpperLimit);
        // check mag limit switches
        if (atUpperLimit && v>0 ) stopElevator();
        if (atLowerLimit && v<0 ) stopElevator();
//        if (i>currentLimit) stopElevator();
        
        SmartDashboard.putNumber("Elevator set", elevatorRight.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Elevator CLE",elevatorRight.getClosedLoopError().getValueAsDouble()); 
        SmartDashboard.putNumber("Elevator MMrun",elevatorRight.getMotionMagicIsRunning().getValueAsDouble());
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
        if (!atLowerLimit) elevatorRight.setControl(voltOutDown);
        else stopElevator();
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
        SmartDashboard.putNumber("Auto pos", pos);
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


/* 
    public void zeroElevator(){
        if(!limitLower.get()){
            while(!limitLower.get()){
                manualUp();
            }
            stopElevator();
        }

        manualDown();
        System.out.println("A1  "+elevatorRight.getVelocity().getValueAsDouble());
        System.out.println("A2  "+elevatorRight.getVelocity().getValueAsDouble());
        
        int i = 0;
        while(elevatorRight.getVelocity().getValueAsDouble()<-0.001 && i>20){
            i++;
            System.out.println("C "+ elevatorRight.getVelocity().getValueAsDouble());
        }
        stopElevator();
        zeroed=true;
        elevatorRight.setPosition(0);
        elevatorFollower.setPosition(0);
        elevatorPos=elevatorRight.getPosition().getValueAsDouble();
        System.out.println("D "+ elevatorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Pos", elevatorPos);
    }

*/


  
  public void configure(){

    cfg.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;

    cfg.MotionMagic.MotionMagicCruiseVelocity=50;
    cfg.MotionMagic.MotionMagicAcceleration=70;
    cfg.MotionMagic.MotionMagicJerk=160;   

    cfg.Slot0.GravityType=GravityTypeValue.Elevator_Static;

    cfg.Slot0.kG=0.495;
    cfg.Slot0.kP=2.0;
    cfg.Slot0.kI=0;
    cfg.Slot0.kD=0;
    cfg.Slot0.kS=0;
    cfg.Slot0.kV=.4;
    cfg.Slot0.kA=0;

    cfg.CurrentLimits.StatorCurrentLimit=80;
    cfg.CurrentLimits.StatorCurrentLimitEnable=true;
    cfg.CurrentLimits.SupplyCurrentLimit=50;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;

    elevatorRight.getConfigurator().apply(cfg);

  }

  public void updateConstants(){
    positionAlgae1= SmartDashboard.getNumber("Elevator positionAlgae1", positionAlgae1);
    positionAlgae2= SmartDashboard.getNumber("Elevator positionAlgae2", positionAlgae2);
    positionNet= SmartDashboard.getNumber("Elevator positionNet", positionNet);
    positionProcessor= SmartDashboard.getNumber("Elevator positionProcessor", positionProcessor);
    positionIntake= SmartDashboard.getNumber("Elevator positionIntake", positionIntake);
    positionCoral1= SmartDashboard.getNumber("Elevator positionCoral1", positionCoral1);
    positionCoral2= SmartDashboard.getNumber("Elevator positionCoral2", positionCoral2);
    positionCoral3= SmartDashboard.getNumber("Elevator positionCoral3", positionCoral3);
    positionCoral4= SmartDashboard.getNumber("Elevator positionCoral4", positionCoral4);
    SmartDashboard.putNumber("Elevator p1", positionCoral1);
    SmartDashboard.putNumber("Elevator p4", positionCoral4);
//    System.out.println("GGG  pos4  "+positionCoral4);
  }

}