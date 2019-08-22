/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToIntakeCommand extends Command {
  private double k_error = 5;
  boolean m_end;
  public MoveArmToIntakeCommand() {
    requires(Robot.m_armSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_end = false;
    if((!Robot.m_oi.hatchMode() && Robot.m_intakeSubsystem.intakeVertical()) || Robot.m_intakeSubsystem.getLimit()){
      if(RobotMap.k_finalRobot){
        Robot.m_armSubsystem.setPosFinal(ArmSubsystem.ticksToDegFinal(RobotMap.k_armIntakeAngle));
      }else{
        Robot.m_armSubsystem.setPos(ArmSubsystem.ticksToDeg(RobotMap.k_armIntakeAngle));
      }
  
      Robot.m_armSubsystem.setStall(false);
    }else{
      m_end = true;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean reached = false;
    if(m_end){
      return true;
    }
    if(RobotMap.k_finalRobot){
      reached = Math.abs(Robot.m_armSubsystem.getAngleFinal() - ArmSubsystem.ticksToDegFinal(RobotMap.k_armIntakeAngle)) <= k_error;
    }else{
      reached = Math.abs(Robot.m_armSubsystem.getAngle() - ArmSubsystem.ticksToDeg(RobotMap.k_armIntakeAngle)) <= k_error;
    }
    return reached;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // SmartDashboard.putBoolean("Arm Ended", m_end);
    Robot.m_armSubsystem.stop();

    Robot.m_armSubsystem.setStall(true);
  }
}
