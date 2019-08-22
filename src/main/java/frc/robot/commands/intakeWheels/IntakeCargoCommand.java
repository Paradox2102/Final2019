/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeWheels;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeCargoCommand extends Command {
  double m_power;
  public IntakeCargoCommand(double power) {
    requires(Robot.m_intakeWheelsSubsystem);

    m_power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_intakeSubsystem.getAngle() < 45){
      Robot.m_intakeWheelsSubsystem.intake(m_power);
    }else{
      Robot.m_intakeWheelsSubsystem.intakeStop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intakeWheelsSubsystem.intakeStop();
  }
}
