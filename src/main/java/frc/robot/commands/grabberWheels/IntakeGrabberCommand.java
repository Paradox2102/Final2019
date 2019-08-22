package frc.robot.commands.grabberWheels;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeGrabberCommand extends Command {
  public IntakeGrabberCommand() {
    requires(Robot.m_grabberWheelSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_grabberWheelSubsystem.intake();
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
    Robot.m_grabberWheelSubsystem.stop();
  }
}
