/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robotCore.Logger;

public class WaitForArmCommand extends Command {
  boolean m_forward;
  double m_angle;
  int m_error;
  public WaitForArmCommand(double angle, int error) {
    m_angle = angle;
    m_error = error;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.Log("Wait for Arm", 3, "init");
    m_forward = m_angle < Robot.m_armSubsystem.getAngle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(RobotMap.k_finalRobot){
      Logger.Log("Wait For Arm", 2, String.format("Cur Angle: %f, Setpoint: %f", Robot.m_armSubsystem.getAngleFinal(), m_angle));
      if(m_forward){
        return Robot.m_armSubsystem.getAngleFinal() - m_error < m_angle;
      }else{
        return Robot.m_armSubsystem.getAngleFinal() + m_error > m_angle;
      }
    }else{
      Logger.Log("Wait For Arm", 2, String.format("Cur Angle: %f, Setpoint: %f", Robot.m_armSubsystem.getAngle(), m_angle));
      if(m_forward){
        return Robot.m_armSubsystem.getAngle() - m_error < m_angle;
      }else{
        return Robot.m_armSubsystem.getAngle() + m_error > m_angle;
      }
    } 
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.Log("Wait for Arm", 3, "End");
  }
}
