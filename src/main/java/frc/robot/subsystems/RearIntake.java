package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix6.controls.VoltageOut;
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

public class RearIntake extends SubsystemBase {

public TalonFX motor = new TalonFX(26);

public DigitalInput limitSwitch= new DigitalInput(2);
private VoltageOut  voltOut= new VoltageOut(.1);
static public boolean releasedRamp=false;


public RearIntake(){


}

    public void periodic(){
        SmartDashboard.putBoolean("Rear Ramp", limitSwitch.get());
        
    }

    public boolean getLimit(){
        return (limitSwitch.get());
    }


    }




