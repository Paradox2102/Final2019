/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CSVWriter;
import frc.lib.CSVWriter.Field;
import frc.robot.Robot;

public class SpeedDriveCommand extends Command {
  double m_speed;

  private Field[] k_fields = {
    new Field("Left Vel", 'f'),
    new Field("Right Vel", 'f')
  };

  CSVWriter writer = new CSVWriter("Vel", k_fields);

  public SpeedDriveCommand(double speed) {
    requires(Robot.m_driveSubsystem);

    m_speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.setSpeed(m_speed, m_speed);
    writer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // SmartDashboard.putNumber("Left Vel", Robot.m_driveSubsystem.getLeftVel());
    // SmartDashboard.putNumber("Right Vel", Robot.m_driveSubsystem.getRightVel());
    writer.write(Robot.m_driveSubsystem.getLeftVel()*-1.0, Robot.m_driveSubsystem.getRightVel());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    writer.finish();
    Robot.m_driveSubsystem.stop();
  }
}
