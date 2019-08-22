/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveIntakePosCommand extends Command {
  private double m_deg;
  private double k_error = 10;
  private boolean m_finalRobot = false;
  public MoveIntakePosCommand(double deg, boolean finalRobot) {
    requires(Robot.m_intakeSubsystem);

    m_deg = deg;
    m_finalRobot = finalRobot;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_intakeSubsystem.setStall(false);
    // if(m_finalRobot){
      // Robot.m_intakeSubsystem.setPosFinal(m_deg);
    // }else{
      Robot.m_intakeSubsystem.setPos(m_deg);
    // }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if(m_finalRobot){
    //   // System.out.println(String.format("Cur Angle:%f, Set Point:%f", Robot.m_intakeSubsystem.getAngle(), m_deg));

    //   return (Math.abs(Robot.m_intakeSubsystem.getAngleFinal() - m_deg) <= k_error);
    // }else{
      // SmartDashboard.putBoolean("Stalling Intake", Math.abs(Robot.m_intakeSubsystem.getAngle() - m_deg) <= k_error);
      // System.out.println(String.format("Cur Angle:%f, Set Point:%f", Robot.m_intakeSubsystem.getAngle(), m_deg));
      return (Math.abs(Robot.m_intakeSubsystem.getAngle() - m_deg) <= k_error);
    // }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // System.out.println("Ended");
    Robot.m_intakeSubsystem.stop();

    Robot.m_intakeSubsystem.setStall(true);
  }
}
