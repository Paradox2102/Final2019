package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ActivatePistonsCommand extends InstantCommand {
  public ActivatePistonsCommand() {
    super();
    requires(Robot.m_climberSubsystem);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_climberSubsystem.set(Value.kForward);
  }

}
