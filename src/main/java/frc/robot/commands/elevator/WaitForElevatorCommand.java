/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robotCore.Logger;

public class WaitForElevatorCommand extends Command {
  double m_pos;
  boolean m_up;
  public WaitForElevatorCommand(double pos) {
    m_pos = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.Log("Wait for Elevator", 3, "Init");
    m_up = m_pos > Robot.m_elevatorSubsystem.getPosRelativeHome();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Logger.Log("Wait for Elevator", 2, String.format("Cur Pos: %d, setpoint: %f, up: %b", Robot.m_elevatorSubsystem.getPosRelativeHome(), m_pos, m_up));
    if(m_up){
      return Robot.m_elevatorSubsystem.getPosRelativeHome() >= m_pos;
    }else{
      return Robot.m_elevatorSubsystem.getPosRelativeHome() < m_pos;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.Log("Wait for Elevator", 3, "End");
  }
}
