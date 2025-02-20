package frc.robot.subsystems;

import org.w3c.dom.ElementTraversal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    final TalonFX elevatorRight = new TalonFX(9); 
    final Follower elevatorLeft = new Follower(10,true );
    final DigitalInput limitLower = new DigitalInput(8);
    final DigitalInput limitUpper = new DigitalInput(9);

    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private MotionMagicTorqueCurrentFOC magicToPos= new MotionMagicTorqueCurrentFOC(0);
    private VoltageOut voltOutUp = new VoltageOut(6);
    private VoltageOut voltOutDown = new VoltageOut(-3);
    
    public double 
            positionAlgae1=.2,positionAlgae2 = 0.3, positionNet, 
            positionCoral1=.5, positionCoral2=.75,
            positionCoral3=1.0,positionCoral4=1.2,
            positionIntake=1.1;
    private double currentLimit=30;
    boolean zeroed=false,atLowerLimit=false,atUpperLimit=false;

// Simulation stuff
 // Simulation classes help us simulate what's going on, including gravity.
 public static final double kElevatorGearing = 9.0;
 public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
 public static final double kCarriageMass = 10; // kg
 public static final double kMinElevatorHeightMeters = 0.0;
 public static final double kMaxElevatorHeightMeters = 1.25;
private final DCMotor m_elevatorGearbox = DCMotor.getFalcon500Foc(2);

 private final ElevatorSim m_elevatorSim =
 new ElevatorSim(
     m_elevatorGearbox,
     kElevatorGearing,
     kCarriageMass,
     kElevatorDrumRadius,
     kMinElevatorHeightMeters,
     kMaxElevatorHeightMeters,
     true,
     0,
     0.01,
     0.0);
     
private final TalonFXSimState elevatorRightSim = elevatorRight.getSimState();

// Create a Mechanism2d visualization of the elevator
private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
private final MechanismLigament2d m_elevatorMech2d =
 m_mech2dRoot.append(
     new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));




public Elevator(){


    SmartDashboard.putNumber("Elevator kG", 0);
    SmartDashboard.putNumber("Elevator kP", 20);
    SmartDashboard.putNumber("Elevator kI", 0);
    SmartDashboard.putNumber("Elevator kD", 0);
    SmartDashboard.putNumber("Elevator kS", 0);
    SmartDashboard.putNumber("Elevator kA", 0);
    SmartDashboard.putNumber("Elevator MMacc", 3);
    SmartDashboard.putNumber("Elevator MMvel", 0.8);
    SmartDashboard.putNumber("Elevator MMjerk", 50);
    SmartDashboard.putNumber("Elevator StatorCL", 60);
    SmartDashboard.putNumber("Elevator SupplyCL", 40);
    SmartDashboard.putNumber("Elevator TorqueCL", 100);

    SmartDashboard.putNumber("Elevator positionAlgae1", positionAlgae1);
    SmartDashboard.putNumber("Elevator positionAlgae2", positionAlgae2);
    SmartDashboard.putNumber("Elevator positionNet", positionNet);
    SmartDashboard.putNumber("Elevator positionIntake", positionIntake);
    SmartDashboard.putNumber("Elevator positionCoral1", positionCoral1);
    SmartDashboard.putNumber("Elevator positionCoral2", positionCoral2);
    SmartDashboard.putNumber("Elevator positionCoral3", positionCoral3);
    SmartDashboard.putNumber("Elevator positionCoral4", positionCoral4);

    SmartDashboard.putData("Elevator Sim", m_mech2d);

    configure();
    stopElevator();
    //  zeroElevator();
  

}

    public void periodic(){
        double v,i;
        v=elevatorRight.getMotorVoltage().getValueAsDouble();
        i=elevatorRight.getStatorCurrent().getValueAsDouble();
        SmartDashboard.putNumber("Elev Pos", elevatorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elev Volt",v);
        SmartDashboard.putNumber("Elev Current", i);

        atLowerLimit=limitLower.get();
        atUpperLimit=limitUpper.get();
        // check mag limit switches
        if (atUpperLimit && v>0 ) stopElevator();
        if (atLowerLimit && v<0 ) stopElevator();
        if (i>currentLimit) stopElevator();
    
        updateTelemetry();
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
        while(!atLowerLimit){
            elevatorRight.setControl(voltOutDown);
        }
        stopElevator();
        zeroed=true;
        elevatorRight.setPosition(0);
    }


 public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(elevatorRightSim.getMotorVoltage() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    elevatorRightSim.setRawRotorPosition(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(elevatorRight.getPosition().getValueAsDouble());
  }

  
  public void configure(){
    double elevatorkG = SmartDashboard.getNumber("Elevator kG", 0);
    double elevatorkP = SmartDashboard.getNumber("Elevator kP", 20);
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

    cfg.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
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
    cfg.Slot0.kV=0;
    cfg.Slot0.kA=elevatorkA;

    cfg.CurrentLimits.StatorCurrentLimit=elevatorStatorCL;
    cfg.CurrentLimits.StatorCurrentLimitEnable=true;
    cfg.CurrentLimits.SupplyCurrentLimit=elevatorSupplyCL;
    cfg.CurrentLimits.SupplyCurrentLimitEnable=true;
    cfg.TorqueCurrent.PeakForwardTorqueCurrent=elevatorTorqueCL;
    cfg.TorqueCurrent.PeakReverseTorqueCurrent=-elevatorTorqueCL;


    elevatorRight.getConfigurator().apply(cfg);

  }



}