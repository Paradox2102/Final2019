package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ReleaseCommand extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ReleaseCommand() {
    super();
    requires(Robot.m_climberSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_climberSubsystem.release();
  }

}
