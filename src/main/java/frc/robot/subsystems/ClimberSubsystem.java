/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ClimberSubsystem extends Subsystem {
  private static final Solenoid climber = new Solenoid(RobotMap.k_climber);
  private static final DoubleSolenoid m_climberDouble = new DoubleSolenoid(RobotMap.k_climberForward, RobotMap.k_climberBackward);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void set(Value value){
    m_climberDouble.set(value);
  }

  public void climb(){
    climber.set(true);
  }

  public void release(){
    climber.set(false);
  }
}
