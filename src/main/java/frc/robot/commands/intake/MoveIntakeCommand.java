package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveIntakeCommand extends Command {
  double m_power;

  public MoveIntakeCommand(double power) {
    requires(Robot.m_intakeSubsystem);

    m_power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_intakeSubsystem.set(m_power);
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
    Robot.m_intakeSubsystem.stop();
  }
}
