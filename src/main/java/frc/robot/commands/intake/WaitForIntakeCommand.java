/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robotCore.Logger;

public class WaitForIntakeCommand extends Command {
  boolean m_forward;
  double m_angle;
  int m_error;
  public WaitForIntakeCommand(double angle, int error) {
    m_angle = angle;
    m_error = error;
  }

  public WaitForIntakeCommand(double angle, int error, int time) {
    m_angle = angle;
    m_error = error;
    setTimeout(time);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.Log("Wait for Intake", 3, "init");
    m_forward = m_angle < Robot.m_intakeSubsystem.getAngle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Logger.Log("Wait For Intake", 2, String.format("Cur Angle: %f, Setpoint: %f", Robot.m_intakeSubsystem.getAngle(), m_angle));
    if(m_forward){
      return Math.abs(Robot.m_intakeSubsystem.getAngle() - m_error) < m_angle || isTimedOut();
    }else{
      return Math.abs(Robot.m_intakeSubsystem.getAngle() + m_error) > m_angle || isTimedOut();
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.Log("Wait for Intake", 3, "End");
  }
}
