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

public class LineFollowerPowerCommand extends Command {
  double m_power;
  double correction = 0.2;
  boolean moved = false;

  public LineFollowerPowerCommand(double power) {
    requires(Robot.m_driveSubsystem);

    m_power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double leftCorrection = 0;
    double rightCorrection = 0;

    int[] lineData = Robot.m_navigator.lineFollower();

    if(lineData[3] < RobotMap.whiteColor && lineData[4] < RobotMap.whiteColor){
      boolean leftCorrected = false;
      boolean rightCorrected = false;

      for(int i=2; i>-1; i--){
        System.out.println(String.format("Id: %d, color: %d", i, lineData[i]));
        if(lineData[i] > RobotMap.whiteColor && !rightCorrected){
          rightCorrection = correction / i+1;
          rightCorrected = true;
        }
      }

      for(int i= 5; i<8; i++){
        // System.out.println(String.format("Id: %d, color: %d", i, lineData[i]));
        if(lineData[i] > RobotMap.whiteColor && !leftCorrected){
          leftCorrection = correction / i+1;
          leftCorrected = true;
        }
      }
    }

    if(Robot.m_driveSubsystem.getLeftVel() > RobotMap.k_stallPower){
      moved = true;
    }

    // System.out.println(String.format("Left Power: %f, Right Power: %f", m_power+leftCorrection, m_power+rightCorrection));

    Robot.m_driveSubsystem.setPower(m_power + leftCorrection, m_power + rightCorrection);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.m_driveSubsystem.getLeftVel() < RobotMap.k_stallPower && moved){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    moved = false;
    Robot.m_driveSubsystem.stop();
  }
}
