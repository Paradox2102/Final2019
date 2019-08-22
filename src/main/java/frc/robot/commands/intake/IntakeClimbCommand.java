package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CSVWriter;
import frc.lib.CSVWriter.Field;
import frc.robot.Robot;

public class IntakeClimbCommand extends Command {
  double m_power;
  double k_pa = .075;
  double m_initPitch;
  double m_pitchTarget = -5;
  Field[] k_fields = {
    new Field("Power", 'f'),
    new Field("Pitch Diff", 'f')
  };
  CSVWriter writer = new CSVWriter("Intake Climb Power", k_fields);
  public IntakeClimbCommand(double power) {
    requires(Robot.m_intakeSubsystem);

    m_pitchTarget = -5;
    m_power = power;
  }

  public IntakeClimbCommand(double power, double pitch) {
    requires(Robot.m_intakeSubsystem);

    m_pitchTarget = pitch;

    m_power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_initPitch = Robot.m_driveSubsystem.getPitch();
    // writer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double pitch = Robot.m_driveSubsystem.getPitch();
    
    double correction = k_pa *(pitch - (m_initPitch + m_pitchTarget));

    Robot.m_intakeSubsystem.set(correction);
    // writer.write(correction, m_initPitch - m_pitchTarget);
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
    // writer.finish();
  }
}
