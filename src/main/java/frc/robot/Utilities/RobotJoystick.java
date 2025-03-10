package frc.robot.Utilities;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotJoystick extends CommandXboxController {
    double scaleFactorSlow=0.333,scaleFactorFast=1;
    double scaleFactorRotateSlow=0.6,scaleFactorRotateFast=1;
    double scaleFactor=scaleFactorFast;
    double deadbandX=0.05,deadbandY=0.05,deadbandRot=0.05;
    private SlewRateLimiter srlx= new SlewRateLimiter(4);
    private SlewRateLimiter srly= new SlewRateLimiter(4);
    double scaleFactorRotate=scaleFactorRotateFast;
    int rotateAxis=4;
   // int rotateAxis=3;

    public RobotJoystick(int port){
        super(port);
        if(Utils.isSimulation())rotateAxis=2;

    }

    // return the x stick value, square it for better low speed control
    public double getX2(){
        double x =Math.pow(getRawAxis(0),2);
        if (x<deadbandX)x=0;
        x=Math.signum(getRawAxis(0))*x;
//        return x*scaleFactor;
        return srlx.calculate(x*scaleFactor);
    }

    // return the y stick value, square it for better low speed control
    public double getY2(){        
        double y = Math.pow(getRawAxis(1),2);
        if (y<deadbandY)y=0.;
        y=Math.signum(getRawAxis(1))*y;
//        return (y*scaleFactor);
        return srly.calculate(y*scaleFactor);
    }
    
    // return the rotate stick value,  TODO - do we ant to sqaure this also?
    public double getRotate(){
//        return Math.signum(controller.getRawAxis(2))*Math.pow(controller.getRawAxis(2),2);
        return getRawAxis(rotateAxis)*scaleFactorRotate;
    }

    public void setSlowScaleFactor(){
        scaleFactor=scaleFactorSlow;
        scaleFactorRotate=scaleFactorRotateSlow;
    }

    public void setFastScaleFactor(){
        scaleFactor=scaleFactorFast;
        scaleFactorRotate=scaleFactorRotateFast;
    }

}
