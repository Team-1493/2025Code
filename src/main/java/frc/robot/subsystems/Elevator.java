package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
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
    private CANcoder encoder = new CANcoder(21);
    private CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    private MotionMagicTorqueCurrentFOC magicToPos= new MotionMagicTorqueCurrentFOC(0);
    private VoltageOut voltOutUp = new VoltageOut(6);
    private VoltageOut voltOutDown = new VoltageOut(-3);
    
    public double pos1=0,pos2=.5,pos3=.75,pos4=1.0,pos5=1.2;
    boolean zeroed=false,atLowerLimit=false,atUpperLimit=false;

// Simulation stuff
 // Simulation classes help us simulate what's going on, including gravity.


 public static final double kElevatorGearing = 30.0;
 public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
 public static final double kCarriageMass = 6.0; // kg
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
     
private final CANcoderSimState encoderSim = encoder.getSimState();
private final TalonFXSimState elevatorRightSim =elevatorRight.getSimState();

// Create a Mechanism2d visualization of the elevator
private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
private final MechanismLigament2d m_elevatorMech2d =
 m_mech2dRoot.append(
     new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));




public Elevator(){

    // Applyy encoder configurations
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint=1;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = 0;//  Set to correct value
    encoder.getConfigurator().apply(cc_cfg);


    // Apply motor configurations

    cfg.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;

    cfg.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID=encoder.getDeviceID();
    cfg.Feedback.SensorToMechanismRatio=1;
   // cfg.Feedback.RotorToSensorRatio=#;  set this value for synching or fusing to remote encoder
    
    cfg.MotionMagic.MotionMagicCruiseVelocity=.5;
    cfg.MotionMagic.MotionMagicAcceleration=2;
    cfg.MotionMagic.MotionMagicJerk=100;   

    cfg.Slot0.kG=2;
    cfg.Slot0.kP=150;
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

    //  TODO  - add remote limit switch using encoder

    elevatorRight.getConfigurator().apply(cfg);

    SmartDashboard.putData("Elevator Sim", m_mech2d);

}

    public void periodic(){
        SmartDashboard.putNumber("Elev Pos-Rem", elevatorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elev Pos-Enc", encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elev PosAbs-Enc", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Elev Volt", elevatorRight.getMotorVoltage().getValueAsDouble());

        atLowerLimit=limitLower.get();
        atUpperLimit=limitUpper.get();
        // check mag limit switches
        if (atUpperLimit && elevatorRight.getMotorVoltage().getValueAsDouble()>0 ) stopElevator();
        if (atLowerLimit && elevatorRight.getMotorVoltage().getValueAsDouble()<0 ) stopElevator();
    
        updateTelemetry();
    }


    public Command manualUpCommand() {
        return runOnce( () -> {manualUp();});
        }

    public void manualUp(){
        SmartDashboard.putString("Man Dir", "up");
        if (!atUpperLimit)  elevatorRight.setControl(voltOutUp);
        else stopElevator();
    }


    public Command manualDownCommand() {
        return runOnce( () -> {manualDown();});
        }

    public void manualDown(){
        SmartDashboard.putString("Man Dir", "down");
        if (!atLowerLimit) elevatorRight.setControl(voltOutDown);
        else stopElevator();
    }


    public Command toPositionCommand(double pos) {
        return runOnce( () -> {toPosition(pos);});
        }

    public void toPosition(double pos){
        SmartDashboard.putNumber("Auto pos", pos);
        elevatorRight.setControl(magicToPos.withPosition(pos));
    }


    public Command stopElevatorCommand() {
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
        encoder.setPosition(0);
        elevatorRight.setPosition(0);
    }


 public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(elevatorRightSim.getMotorVoltage() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    encoderSim.setRawPosition(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(encoder.getPosition().getValueAsDouble()*40);
  }
}