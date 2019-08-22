/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class MoveElevatorToIntakeCommand extends Command {
  boolean m_up;
  boolean m_end;
  public MoveElevatorToIntakeCommand() {
    requires(Robot.m_intakeSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double power = 0.75;
    m_up = RobotMap.k_elevatorIntakeHeight > Robot.m_elevatorSubsystem.getPosRelativeHome();

    if(!m_up){
      power = 0.5;
    }

    m_end = false;
    if(!Robot.m_oi.hatchMode() && Robot.m_intakeSubsystem.getLimit()){
      Robot.m_elevatorSubsystem.set(power);
    }else{
      m_end = true;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_end){
      return true;
    }
    if(m_up){
      return Robot.m_elevatorSubsystem.getPosRelativeHome() > RobotMap.k_elevatorIntakeHeight;
    }else{
      return Robot.m_elevatorSubsystem.getPosRelativeHome() < RobotMap.k_elevatorIntakeHeight;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevatorSubsystem.stop();
  }
}
