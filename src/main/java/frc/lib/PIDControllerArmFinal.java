package frc.lib;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import frc.robot.subsystems.ArmSubsystem;

public class PIDControllerArmFinal extends PIDController{
    public PIDControllerArmFinal(double Kp, double Ki, double Kd, double Kf, PIDSource source,
        PIDOutput output){
        super(Kp, Ki, Kd, Kf, source, output);
    }

    public double calculateFeedForward(){
        return -getF() * Math.cos(Math.toRadians(ArmSubsystem.ticksToDegFinal(getSetpoint())));
    }
}