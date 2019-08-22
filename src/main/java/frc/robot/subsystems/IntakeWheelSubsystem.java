package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class IntakeWheelSubsystem extends Subsystem {
  private static final TalonSRX m_intakeWheels = new TalonSRX(RobotMap.k_intakeWheels);
  private static final TalonSRX m_intakeWheelsFollower = new TalonSRX(RobotMap.k_intakeWheelsFollower);

  public IntakeWheelSubsystem(){
    m_intakeWheels.setInverted(true);
    m_intakeWheelsFollower.setInverted(true);
    
    m_intakeWheelsFollower.set(ControlMode.Follower, RobotMap.k_intakeWheels);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  
  public void intake(double power){
    m_intakeWheels.set(ControlMode.PercentOutput, power);
  }

  public void intakeStop(){
    m_intakeWheels.set(ControlMode.PercentOutput, 0);
  }
}
