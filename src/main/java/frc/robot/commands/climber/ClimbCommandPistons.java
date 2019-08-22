package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ClimbCommandPistons extends InstantCommand {
  public ClimbCommandPistons() {
    super();
    requires(Robot.m_climberSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_climberSubsystem.climb();
  }

}
