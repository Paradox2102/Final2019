package frc.lib;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class PIDControllerElevator extends PIDController{
    public PIDControllerElevator(double Kp, double Ki, double Kd, double Kf, PIDSource source,
        PIDOutput output){
        super(Kp, Ki, Kd, Kf, source, output);
    }

    public double calculateFeedForward(){
        return getF();
    }
}