package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StallArmCommand extends Command {
  boolean m_final;

  public StallArmCommand(boolean finalRobot) {
    requires(Robot.m_armSubsystem);

    m_final = finalRobot;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // SmartDashboard.putString("Stalling", "We are stalling");
    // SmartDashboard.putBoolean("Stall Value", Robot.m_armSubsystem.stall());
    double lastArm = Robot.m_armSubsystem.lastPos();
    if(Robot.m_armSubsystem.stall()){
      if(m_final){
        Robot.m_armSubsystem.setPosFinal(lastArm);
      }else{
        Robot.m_armSubsystem.setPos(lastArm);
      }
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
    // SmartDashboard.putString("Stalling", "We are not stalling");
    Robot.m_armSubsystem.stop();
  }
}
