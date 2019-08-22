/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TeleopElevatorCommand extends Command {
  private final static double k_deadban = 0.15;
  private final static double k_maxPower = 0.75;
  public TeleopElevatorCommand() {
    requires(Robot.m_elevatorSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double elevatorPower = Robot.m_oi.getElevatorY();
    elevatorPower = elevatorPower * elevatorPower * elevatorPower;
    if(Math.abs(elevatorPower) > k_deadban){
      //TODO: take logging out
      SmartDashboard.putNumber("Elevator Power", elevatorPower * k_maxPower);
      Robot.m_elevatorSubsystem.set(elevatorPower * k_maxPower);
    }else{
      Robot.m_elevatorSubsystem.stop();
    }
    // SmartDashboard.putNumber("Elevator Pos", Robot.m_elevatorSubsystem.getPos());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevatorSubsystem.stop();
    // SmartDashboard.putNumber("Elevator Pos", Robot.m_elevatorSubsystem.getPos());
  }
}
