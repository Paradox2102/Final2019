package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class GrabberSubsystem extends Subsystem {
  private final static DoubleSolenoid m_grabber = new DoubleSolenoid(RobotMap.k_grabberForward, RobotMap.k_grabberReverse);
  // private final static DigitalInput m_hatchLimitSwitch = new DigitalInput(RobotMap.k_hatchLimitSwitch);
  public Value lastValue;
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void open(){
    m_grabber.set(Value.kForward);
    lastValue = Value.kForward;
  }

  public void close(){
    m_grabber.set(Value.kReverse);
    lastValue = Value.kReverse;
  }

  public void toggle(){
    
    if(m_grabber.get() == Value.kForward){
      m_grabber.set(Value.kReverse);
    }else{
      m_grabber.set(Value.kForward);
    }
  }
}
