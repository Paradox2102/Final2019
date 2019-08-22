/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StallDriveCommand extends Command {
  boolean m_driven = false;
  double m_power;
  double k_error = 5;
  public StallDriveCommand(double power) {
    requires(Robot.m_driveSubsystem);

    m_power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.setPower(m_power, m_power);

    setTimeout(0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double vel = Robot.m_driveSubsystem.getLeftVel();
    m_driven = Math.abs(vel) > k_error*1.5;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
      double vel = Robot.m_driveSubsystem.getLeftVel();
      // SmartDashboard.putBoolean("Stall Drive", (Math.abs(vel) <= k_error) && m_driven);
      // SmartDashboard.putNumber("Stall vel", vel);
      // SmartDashboard.putBoolean("timed out", isTimedOut());
      return ((Math.abs(vel) <= k_error) && m_driven) || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveSubsystem.stop();
  }
}
