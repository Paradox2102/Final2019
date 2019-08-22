/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robotCore.Logger;

public class TeleopDriveCommand extends Command {
  private double k_maxPower = 0.75;
  // public static boolean m_disable = false;
  public TeleopDriveCommand() {
    requires(Robot.m_driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // if(m_disable){
      Robot.m_driveSubsystem.disablePID();
    // }
    Logger.Log("Teleop Drive", 3, "Init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double x = Robot.m_oi.getDriveX();
    double y = Robot.m_oi.getDriveY();

    x = x * x * x;
    y = y * y * y;

    if(Robot.m_oi.getDriveThrottle() < 0){
      y = -y;
    }

    // if(!m_disable){
      Robot.m_driveSubsystem.setPower((y + x)*k_maxPower, (y - x)*k_maxPower);
    // }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.Log("Teleop Drive", 3, "End");
    Robot.m_driveSubsystem.stop();
  }
}
