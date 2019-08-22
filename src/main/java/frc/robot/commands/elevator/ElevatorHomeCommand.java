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

public class ElevatorHomeCommand extends Command {
  public ElevatorHomeCommand() {
    requires(Robot.m_elevatorSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.Log("Elevator Home", 3, "Init");
    Robot.m_elevatorSubsystem.set(-0.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.m_elevatorSubsystem.getRevLimit();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.Log("Elevator Home", 3, "End");
    Robot.m_elevatorSubsystem.stop();
  }
}
