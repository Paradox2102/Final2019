/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveArmCommand extends Command {
  private boolean m_forward;
  public MoveArmCommand(boolean forward) {
    requires(Robot.m_armSubsystem);

    m_forward = forward;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double power = Robot.m_oi.getElevatorThrottle();

    if(m_forward){
      Robot.m_armSubsystem.setPower(power, false);
    }else{
      Robot.m_armSubsystem.setPower(-power, false);
    }

    // SmartDashboard.putNumber("Arm Power", power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_armSubsystem.stop();
  }
}
