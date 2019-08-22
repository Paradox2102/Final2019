/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class GrabberWheelSubsystem extends Subsystem {
  private final static TalonSRX m_wheels = new TalonSRX(RobotMap.k_grabberWheels);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void intake(){
    m_wheels.set(ControlMode.PercentOutput, 1);
  }

  public void outtake(){
    m_wheels.set(ControlMode.PercentOutput, -1);
  }

  public void stop(){
    m_wheels.set(ControlMode.PercentOutput, 0);
  }
}
