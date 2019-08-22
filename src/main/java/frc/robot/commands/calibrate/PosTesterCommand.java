/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.calibrate;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CSVWriter;
import frc.lib.CSVWriter.Field;
import frc.robot.Robot;
import frc.robot.PositionTracker.PositionContainer;

public class PosTesterCommand extends Command {
  double m_leftEncoder;
  double m_rightEncoder;

  private Field[] k_fields = {
    new Field("x", 'f'),
    new Field("y", 'f'),
    new Field("Left Encoder", 'f'),
    new Field("Right Encoder", 'f')
  };

  CSVWriter writer = new CSVWriter("Front Camera Target Calibration", k_fields);
  
  public PosTesterCommand() {
    requires(Robot.m_driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.setXY(0, 0);
    Robot.m_driveSubsystem.setAngle(270);
    Robot.m_driveSubsystem.setGyro(270);

    writer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    PositionContainer pos = Robot.m_driveSubsystem.getPos();

    writer.write(pos.x, pos.y, Robot.m_driveSubsystem.getLeftPos(), Robot.m_driveSubsystem.getRightPos());
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
  }
}
