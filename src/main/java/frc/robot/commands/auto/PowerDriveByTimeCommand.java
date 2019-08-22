/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robotCore.Logger;

public class PowerDriveByTimeCommand extends Command {
  double m_power;
  double m_time;
  public PowerDriveByTimeCommand(double power, double time) {
    requires(Robot.m_driveSubsystem);
    m_power = power;
    m_time = time;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.Log("Power Drive By Time", 3, "Init");
    setTimeout(m_time);
    Robot.m_driveSubsystem.disablePID();
    Robot.m_driveSubsystem.setPower(m_power, m_power);
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
    Robot.m_driveSubsystem.stop();
    Logger.Log("Power Drive By Time", 3, "End");
  }

  @Override
  protected void interrupted(){
    Logger.Log("Power Drive By Time", 3, "Interupted");
    end();
  }
}
