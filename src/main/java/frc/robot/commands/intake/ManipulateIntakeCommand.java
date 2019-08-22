/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ManipulateIntakeCommand extends Command {
  double m_deg;
  double k_power = 0.5;
  boolean m_hatchMode;
  boolean m_finalRobot = false;
  double k_error = 2;

  public ManipulateIntakeCommand(double deg, boolean finalRobot) {
    requires(Robot.m_intakeSubsystem);

    m_deg = deg;
    m_finalRobot = finalRobot;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_hatchMode = Robot.m_oi.hatchMode();

    if(m_hatchMode){
      Robot.m_intakeSubsystem.set(-k_power);
    }else{
      // SmartDashboard.putBoolean("Going to intake pos", (Robot.m_intakeSubsystem.intakeVertical() || Robot.m_intakeSubsystem.getLimit()));
      if(Robot.m_intakeSubsystem.intakeVertical() || Robot.m_intakeSubsystem.getLimit()){
        Robot.m_intakeSubsystem.setPos(m_deg);
      }else{
        Robot.m_intakeSubsystem.setPos(RobotMap.k_intakeUpPos);
      }
    }

    Robot.m_armSubsystem.setStall(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_hatchMode){
      return Robot.m_intakeSubsystem.getLimit();
    }else{
      if(m_finalRobot){
        return (Math.abs(Robot.m_intakeSubsystem.getAngleFinal() - m_deg) <= k_error);
      }else{
        return (Math.abs(Robot.m_intakeSubsystem.getAngle() - m_deg) <= k_error);
      }
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_intakeSubsystem.stop();

    Robot.m_armSubsystem.setStall(true);
  }
}
