/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveArmOverrideCommand extends Command {
  double m_power;
  public MoveArmOverrideCommand(double power) {
    requires(Robot.m_armSubsystem);

    m_power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_armSubsystem.checkDisable(false);
    Robot.m_armSubsystem.powerOverride(m_power);
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
    Robot.m_armSubsystem.stop();

    if(Robot.m_armSubsystem.getPos() > 10){
      Robot.m_armSubsystem.enableMotor();
    }

    Robot.m_armSubsystem.checkDisable(true);
  }
}
