/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.elevator.MoveElevatorPosCommand.StallCase;

public class ElevatorLowPlacementCommand extends Command {
  double k_elevatorPlacementPos = 150;//657
  boolean end = false;
  StallCase m_stallCase;
  long m_startTime;
  long k_stallTime = 250;
  int k_error = 100;
  public ElevatorLowPlacementCommand() {
    requires(Robot.m_elevatorSubsystem);
  }
  protected void initialize() {
    end = false;
    m_stallCase = StallCase.checkEnd;
    m_startTime = System.currentTimeMillis();
    if(Robot.m_elevatorSubsystem.getRevLimit()){
      Robot.m_elevatorSubsystem.setPos(k_elevatorPlacementPos);
    }else{
      end = true;
    }

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    switch (m_stallCase){
      case checkEnd:
        boolean finished = Robot.m_elevatorSubsystem.getPosRelativeHome() > k_elevatorPlacementPos - k_error;
        if(finished){
          m_startTime = System.currentTimeMillis();
          m_stallCase = StallCase.stalling;
        }
        break;
      case stalling:
        if(m_startTime + k_stallTime < System.currentTimeMillis()){
          Robot.m_elevatorSubsystem.brake();
          m_stallCase = StallCase.braked;
          m_startTime = System.currentTimeMillis();
        }
        break;
      case braked:
        if(m_startTime + k_stallTime < System.currentTimeMillis()){
          return true;
        }
        break;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevatorSubsystem.stop();
    Robot.m_elevatorSubsystem.setMaxOutput(1);
  }
}