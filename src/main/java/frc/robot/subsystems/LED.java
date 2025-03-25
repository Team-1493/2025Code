package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LED extends SubsystemBase {
    private final AddressableLED m_led1 ;
    private final AddressableLEDBuffer m_ledBuffer1 =new 
            AddressableLEDBuffer(59);
  
    // Our LED strip has a density of 120 LEDs per meter

   // private static final Distance kLedSpacing = Meters.of(1 / 120.0);
    int enabledColor = 0, elvatAtPosColor = 1, hasCoralColor = 2,
        endGameColor = 3, disabledColor = 4;
    int elevatorCounter=0;
        private final LEDPattern m_blue = LEDPattern.solid(Color.kBlue);
    
        Color red = new Color(0,255,0);
        private final LEDPattern m_red = LEDPattern.solid(red);
        
    Color green = new Color(255,0,0);
    private final LEDPattern m_green = LEDPattern.solid(green);

    private final LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern m_violetred = LEDPattern.solid(Color.kPaleVioletRed);
    private ArrayList<LEDPattern> basePatterns = new ArrayList<LEDPattern>();
    public static boolean   hasCoral =false, 
                            enabled = false,
                            endgame = false,
                            elevatorAtPos = false, 
                            nearTarget = false,
                            prevEnabled = false,
                            prevHasCoral = false,
                            prevElevatAtPos = false,
                            prevNearTarget = false,
                            prevEndGame=false;

                            public int baseColor=disabledColor;   
    private Trigger endgameTrigger; 
    
    private double matchTime=0;

  
public LED(){

    m_led1= new AddressableLED(5);
    m_led1.setLength(m_ledBuffer1.getLength());

    basePatterns.add(m_blue);
    basePatterns.add(m_red);
    basePatterns.add(m_green);
    basePatterns.add(m_yellow);
    basePatterns.add(m_violetred);
    

    basePatterns.get(disabledColor).applyTo(m_ledBuffer1);
    m_led1.setData(m_ledBuffer1);
    m_led1.start();

    endgameTrigger = new Trigger( ()-> getEndGameState());
    

   

//    enabledTrigger = new Trigger(this:getEnabledState());


}

    public void periodic(){

        matchTime=DriverStation.getMatchTime();
        if(matchTime>-0.1) SmartDashboard.putNumber("MatchTime",(int)matchTime);

        if(matchTime<20  && matchTime>-0.1 &&  DriverStation.isTeleop()) 
            endgame=true;

        if(prevEnabled && !enabled) setBaseColor(disabledColor);
        else if(!prevEnabled && enabled) setBaseColor(enabledColor);
        prevEnabled=enabled;

        if(!prevHasCoral && hasCoral) setHasCoralColor();
        else if (prevHasCoral && !hasCoral) setBaseColor(baseColor);
        prevHasCoral = hasCoral;

         if(!prevElevatAtPos && elevatorAtPos  ){
                setElevatorAtPosColor();
                elevatorCounter=0;
         }
         
            
        if (prevElevatAtPos && elevatorAtPos) elevatorCounter = elevatorCounter+1;
         
         if(elevatorCounter>50){
            setBaseColor(baseColor);
         }

        prevElevatAtPos=elevatorAtPos;


        if(!prevEndGame && endgame) setBaseColor(endGameColor);
         prevEndGame=endgame;

    }


    private void setBaseColor(int i){
        if(!hasCoral){
        (basePatterns.get(i)).applyTo(m_ledBuffer1);
        m_led1.setData(m_ledBuffer1);
        m_led1.start();
        baseColor=i;
        }
        else setHasCoralColor();
    }

    private void setHasCoralColor(){
        basePatterns.get(hasCoralColor).applyTo(m_ledBuffer1);
        m_led1.setData(m_ledBuffer1);
        m_led1.start();
    }
    

    private void setElevatorAtPosColor(){
        basePatterns.get(elvatAtPosColor).applyTo(m_ledBuffer1);

        m_led1.setData(m_ledBuffer1);
        m_led1.start();
    }


    private void setNearTargetBlink(){
        (basePatterns.get(disabledColor).blink(Seconds.of(0.25))).
            applyTo(m_ledBuffer1);
        m_led1.setData(m_ledBuffer1);
        m_led1.start();
    }



   

    private boolean getEndGameState(){
        return endgame;
    }


    }


    



