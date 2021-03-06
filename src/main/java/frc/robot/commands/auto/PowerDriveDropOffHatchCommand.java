/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PowerDriveDropOffHatchCommand extends Command {
  double m_power;
  boolean m_releaseHatch = false;
  public PowerDriveDropOffHatchCommand(double power) {
    requires(Robot.m_driveSubsystem);
    requires(Robot.m_armSubsystem);
    m_power = power;
  }

  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.disablePID();
  }

  @Override
  protected void execute() {
    Robot.m_driveSubsystem.setPower(m_power, m_power);
  }

  @Override
  protected boolean isFinished() {
    double time = timeSinceInitialized();
    if(time >= 1.0){
      m_releaseHatch = false;
      return true;
    }
    m_releaseHatch = true;
    return Robot.m_armSubsystem.hasHatch();
  }

  @Override
  protected void end() {
    Robot.m_driveSubsystem.stop();
    if(m_releaseHatch){
      Robot.m_grabberSubsystem.close();
    }
  }
}
