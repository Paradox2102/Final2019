/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.PositionTracker.PositionContainer;

public class LineFollowerSpeedCommand extends Command {
  double m_speed;
  double m_trarvelDist;
  double m_angle;

  double k_correction = 200;
  double m_leftCorrection = 0;
  double m_rightCorrection = 0;

  PositionContainer m_startPos;
  int m_overShootIndex;

  boolean m_overShot = false;
  boolean m_driveStraight = false;

  double k_errorDistort = 10.0;

  public LineFollowerSpeedCommand(double speed, double travelDist, double angle) {
    requires(Robot.m_driveSubsystem);

    m_speed = speed;
    m_trarvelDist = travelDist;
    m_angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_startPos = Robot.m_driveSubsystem.getPos();

    int[] lineData = Robot.m_navigator.lineFollower();

    if(lineData[3] < RobotMap.whiteColor && lineData[4] < RobotMap.whiteColor){
      boolean leftChecked = false;
      boolean rightChecked = false;

      for(int i=2; i>-1; i--){
        // System.out.println(String.format("Id: %d, color: %d", i, lineData[i]));
        if(lineData[i] > RobotMap.whiteColor && !rightChecked){
          rightChecked = true;
          m_overShootIndex = 7 - i;

          m_rightCorrection = k_correction / (i + 1);
        }
      }

      for(int i=5; i<8; i++){
        // System.out.println(String.format("Id: %d, color: %d", i, lineData[i]));
        if(lineData[i] > RobotMap.whiteColor && !leftChecked){
          leftChecked = true;
          m_overShootIndex = 7 - i;

          m_leftCorrection = k_correction / (8 - i);
        }
      }
    }else{
      m_driveStraight = true;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    int[] lineData = Robot.m_navigator.lineFollower();

    if(!m_overShot && !m_driveStraight){
        Robot.m_driveSubsystem.setSpeed(m_speed + m_leftCorrection - m_rightCorrection, m_speed + m_rightCorrection - m_leftCorrection);
        if(lineData[m_overShootIndex] > RobotMap.whiteColor){
          m_overShot = true;
        }
    }else if(m_overShot){
      Robot.m_driveSubsystem.setSpeed(m_speed + m_rightCorrection - m_leftCorrection, m_speed + m_leftCorrection - m_rightCorrection);
    }else if(m_driveStraight){
      double curAngle = Robot.m_driveSubsystem.getAngle();

      double error = k_correction * ((m_angle - curAngle)/k_errorDistort);

      Robot.m_driveSubsystem.setSpeed(m_speed - error, m_speed + error);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    PositionContainer pos = Robot.m_driveSubsystem.getPos();

    // double distTrav = Math.sqrt(pos.x * pos.x + pos.y * )
    if((pos.y*12.0) - (m_startPos.y*12) >= m_trarvelDist){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_leftCorrection = 0;
    m_rightCorrection = 0;
    m_overShot = false;
    m_driveStraight = false;
    Robot.m_driveSubsystem.stop();
  }
}
