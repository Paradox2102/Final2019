/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.calibrate;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CalibrateTicksPerFootCommand extends Command {
  double m_leftStartPosSpark;
  double m_rightStartPosSpark;
  double m_leftStartPosGrey;
  double m_rightStartPosGrey;

  public CalibrateTicksPerFootCommand() {
    requires(Robot.m_driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.setCoastMode();
    m_leftStartPosSpark = Robot.m_driveSubsystem.getLeftPos();
    m_rightStartPosSpark = Robot.m_driveSubsystem.getRightPos();
    m_leftStartPosGrey = Robot.m_driveSubsystem.getLeftPosGrey();
    m_rightStartPosGrey = Robot.m_driveSubsystem.getRightPosGrey();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    double leftPosSpark = Robot.m_driveSubsystem.getLeftPos() -  m_leftStartPosSpark;
    double rightPosSpark = Robot.m_driveSubsystem.getRightPos() - m_rightStartPosSpark;
    double leftPosGrey = Robot.m_driveSubsystem.getLeftPosGrey() -  m_leftStartPosGrey;
    double rightPosGrey = Robot.m_driveSubsystem.getRightPosGrey() - m_rightStartPosGrey;
    System.out.println("Left Spark: " + leftPosSpark);
    System.out.println("Right Spark: " + rightPosSpark);

    System.out.println("Left Grey: " + leftPosGrey);
    System.out.println("Right Grey: " + rightPosGrey);
  }
}
