/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class StallIntakeCommand extends Command {
  boolean m_final;

  public StallIntakeCommand(boolean finalRobot) {
    requires(Robot.m_intakeSubsystem);

    m_final = finalRobot;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println("Init");
    double deg = Robot.m_intakeSubsystem.lastPos();
    if(Robot.m_intakeSubsystem.stall()){
      // System.out.println("Stall");

      if(m_final){
        Robot.m_intakeSubsystem.setPosFinal(deg);
      }else{
        Robot.m_intakeSubsystem.setPos(deg);
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
  }
}
