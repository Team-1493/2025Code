package frc.robot.subsystems;

import org.w3c.dom.ElementTraversal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Elevator extends SubsystemBase{
    final TalonFX elevatorRight = new TalonFX(22); 
    final TalonFX elevatorFollower = new TalonFX(21);
    final DigitalInput limitLower = new DigitalInput(0);
    final DigitalInput limitUpper = new DigitalInput(3);

    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private MotionMagicVoltage  magicToPos= new MotionMagicVoltage(0);
    private VoltageOut voltOutUp = new VoltageOut(.75);
    private VoltageOut voltOutDown = new VoltageOut(-.20);
    
    public double 
            positionAlgae1=10,positionAlgae2 = 15, positionNet=1, 
            positionCoral1=5, positionCoral2=12,
            positionCoral3=18,positionCoral4=26,
            positionIntake=.03;
    private double currentLimit=100;
    public double elevatorPos=0;
    boolean zeroed=false,atLowerLimit=false,atUpperLimit=false;

     
private final TalonFXSimState elevatorRightSim = elevatorRight.getSimState();




public Elevator(){

    elevatorFollower.setControl(new Follower(22,true));

    SmartDashboard.putNumber("Elevator kG", 0.465);
    SmartDashboard.putNumber("Elevator kP", 2);
    SmartDashboard.putNumber("Elevator kI", 0);
    SmartDashboard.putNumber("Elevator kD", 0);
    SmartDashboard.putNumber("Elevator kS", 0);
    SmartDashboard.putNumber("Elevator kA", 0);
    SmartDashboard.putNumber("Elevator MMacc", 70);
    SmartDashboard.putNumber("Elevator MMvel", 50);
    SmartDashboard.putNumber("Elevator MMjerk", 160);
    SmartDashboard.putNumber("Elevator StatorCL", 60);
    SmartDashboard.putNumber("Elevator SupplyCL", 40);
    SmartDashboard.putNumber("Elevator TorqueCL", 100);
    System.out.println("************************************************"+positionCoral1);
    SmartDashboard.putNumber("Elevator positionAlgae1", positionAlgae1);
    SmartDashboard.putNumber("Elevator positionAlgae2", positionAlgae2);
    SmartDashboard.putNumber("Elevator positionNet", positionNet);
    SmartDashboard.putNumber("Elevator positionIntake", positionIntake);
    SmartDashboard.putNumber("Elevator positionCoral1", positionCoral1);
    SmartDashboard.putNumber("Elevator positionCoral2", positionCoral2);
    SmartDashboard.putNumber("Elevator positionCoral3", positionCoral3);
    SmartDashboard.putNumber("Elevator positionCoral4", positionCoral4);

    
    configure();
    stopElevator();
  

}

    public void periodic(){
        if (!Robot.enabled) zeroed=false;
        if(!zeroed)zeroElevator();
        atLowerLimit=!limitLower.get();
        atUpperLimit=!limitUpper.get();

        double v,i,c;
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


    public Command StopElevator() {
        return runOnce( () -> {stopElevator();});
        }

    public void stopElevator(){
        SmartDashboard.putString("Man Dir", "stop");
        elevatorRight.stopMotor();
    }


    public void zeroElevator(){
        double p;
        if(!limitLower.get()){
            while(!limitLower.get()){
                manualUp();
            }
            stopElevator();
        }

        manualDown();
        Timer.delay(.02);
        while(elevatorRight.getVelocity().getValueAsDouble()<-0.001){
        }
        stopElevator();
        zeroed=true;
        elevatorRight.setPosition(0);
    }




  
  public void configure(){
    double elevatorkG = SmartDashboard.getNumber("Elevator kG", 0.56);
    double elevatorkP = SmartDashboard.getNumber("Elevator kP", 0);
    double elevatorkI = SmartDashboard.getNumber("Elevator kI", 0);
    double elevatorkD = SmartDashboard.getNumber("Elevator kD", 0);
    double elevatorkS = SmartDashboard.getNumber("Elevator kS", 0);
    double elevatorkA = SmartDashboard.getNumber("Elevator kA", 0);

    double elevatorMMacc = SmartDashboard.getNumber("Elevator MMacc", 3);
    double elevatorMMvel = SmartDashboard.getNumber("Elevator MMvel", 0.8);
    double elevatorMMjerk = SmartDashboard.getNumber("Elevator MMjerk", 50);
    double elevatorStatorCL = SmartDashboard.getNumber("Elevator StatorCL", 60);
    double elevatorSupplyCL = SmartDashboard.getNumber("Elevator SupplyCL", 40);
    double elevatorTorqueCL = SmartDashboard.getNumber("Elevator TorqueCL", 100);
    positionAlgae1= SmartDashboard.getNumber("Elevator positionAlgae1", positionAlgae1);
    positionAlgae2= SmartDashboard.getNumber("Elevator positionAlgae2", positionAlgae2);
    positionNet= SmartDashboard.getNumber("Elevator positionNet", positionNet);
    positionIntake= SmartDashboard.getNumber("Elevator positionIntake", positionIntake);
    positionCoral1= SmartDashboard.getNumber("Elevator positionCoral1", positionCoral1);
    positionCoral2= SmartDashboard.getNumber("Elevator positionCoral2", positionCoral2);
    positionCoral3= SmartDashboard.getNumber("Elevator positionCoral3", positionCoral3);
    positionCoral4= SmartDashboard.getNumber("Elevator positionCoral4", positionCoral4);


    // Apply motor configurations

    
    cfg.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;
    
    cfg.MotionMagic.MotionMagicCruiseVelocity=elevatorMMvel;
    cfg.MotionMagic.MotionMagicAcceleration=elevatorMMacc;
    cfg.MotionMagic.MotionMagicJerk=elevatorMMjerk;   

    cfg.Slot0.GravityType=GravityTypeValue.Elevator_Static;
    cfg.Slot0.kG=elevatorkG;
    cfg.Slot0.kP=elevatorkP;
    cfg.Slot0.kI=elevatorkI;
    cfg.Slot0.kD=elevatorkD;
    cfg.Slot0.kS=elevatorkS;
    cfg.Slot0.kV=.4;
    cfg.Slot0.kA=elevatorkA;

    cfg.CurrentLimits.StatorCurrentLimit=elevatorStatorCL;
    cfg.CurrentLimits.StatorCurrentLimitEnable=true;
    cfg.CurrentLimits.SupplyCurrentLimit=elevatorSupplyCL;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;


    elevatorRight.getConfigurator().apply(cfg);

  }



}