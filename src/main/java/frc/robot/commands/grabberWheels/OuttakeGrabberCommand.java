/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabberWheels;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OuttakeGrabberCommand extends Command {
  public OuttakeGrabberCommand() {
    requires(Robot.m_grabberWheelSubsystem);
  }

  public OuttakeGrabberCommand(double time){
    setTimeout(time);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_grabberWheelSubsystem.outtake();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_grabberWheelSubsystem.stop();
  }
}
