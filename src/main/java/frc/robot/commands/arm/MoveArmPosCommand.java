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
import frc.robotCore.Logger;

public class MoveArmPosCommand extends Command {
  private double m_deg;
  private double k_error = 7;
  private double m_adjustment;
  private boolean m_final = false;
  private boolean m_driverInput;
  private ArmPos m_armPos;

  public enum ArmPos{
    Up, Backward, Forward, Intake, FeederForward, FeederBackward, CargoForward, CargoBackward, HatchCameraAvoid
  } 
  public MoveArmPosCommand(ArmPos armPos, boolean finalRobot) {
    requires(Robot.m_armSubsystem);

    m_armPos = armPos;
    m_final = finalRobot;
    m_driverInput = true;
  }

  public MoveArmPosCommand(ArmPos armPos, boolean finalRobot, boolean driverInput) {
    requires(Robot.m_armSubsystem);

    m_armPos = armPos;
    m_final = finalRobot;
    m_driverInput = driverInput;
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.Log("MoveArmPosCommand", 3, "initialize");

    if(!m_driverInput){
      if(m_armPos == ArmPos.Up){
        m_deg = 90;
      }else if(m_armPos == ArmPos.Backward){
        m_deg = 180;
      }else if(m_armPos == ArmPos.Forward){
        if(Robot.m_armSubsystem.hasHatch()){
          m_deg = 0;
        }else{
          m_deg = -8;
        }
      }
    }else{
      if(m_armPos == ArmPos.Up){
        m_deg = 90;
      }else if(m_armPos == ArmPos.Forward){
        if(Robot.m_oi.hatchMode()){
          if(Robot.m_armSubsystem.hasHatch()){
            m_deg = 0;
          }else{
            m_deg = -8;
          }
        }else{
          if(Robot.m_elevatorSubsystem.getPosRelativeHome() > RobotMap.k_rocketCargoLevel2 + 1000){
            m_deg = 10;
          }else{
            m_deg = RobotMap.k_cargoRocketAngleFront;
          }
        }
      }else if(m_armPos == ArmPos.Backward){
        if(Robot.m_oi.hatchMode()){
          if(Robot.m_armSubsystem.hasHatch()){
            m_deg = 180;
          }else{
            m_deg = 170;
          }
        }else{
          if(Robot.m_elevatorSubsystem.getPosRelativeHome() > RobotMap.k_rocketCargoLevel2 + 1000){
            m_deg = 175;
          }else{
            m_deg = RobotMap.k_cargoRocketAngleBack;
          }
        }
      }
    }

    if(m_armPos == ArmPos.Intake){
      m_deg = RobotMap.k_armIntakeAngle;
    }else if(m_armPos == ArmPos.FeederForward){
      m_deg = RobotMap.k_armFeederAngleForward;
    }else if(m_armPos == ArmPos.FeederBackward){
      m_deg = RobotMap.k_armFeederAngleBackward;
    }else if(m_armPos == ArmPos.CargoForward){
      m_deg = RobotMap.k_cargoShipAngleFront;
    }else if(m_armPos == ArmPos.CargoBackward){
      m_deg = RobotMap.k_cargoShipAngleBack;
    }else if(m_armPos == ArmPos.HatchCameraAvoid){
      m_deg = RobotMap.k_hatchCameraAvoid;
    }

    if(m_final){
      Robot.m_armSubsystem.setPosFinal(m_deg);
    }else{
      Robot.m_armSubsystem.setPos(m_deg);
    }
    // m_adjustment = 0;

    // // System.out.println("Move Arm Pos Init");
    // if((m_deg == 90 || Robot.m_oi.hatchMode() || !m_driverInput)){
    //   Logger.Log("MoveArmPosCommand", 2, "Driver Input");
    //   if(m_final){
    //     Robot.m_armSubsystem.setPosFinal(m_deg);
    //   }else{
    //     Robot.m_armSubsystem.setPos(m_deg);
    //   }
    // }else{
    //   if(m_deg < 10){
    //     m_adjustment += 10;
    //   }else if(m_deg > 100){
    //     m_adjustment -= 10;
    //   }
    //   if(m_final){
    //     Robot.m_armSubsystem.setPosFinal(m_deg + m_adjustment);
    //   }else{
    //     Robot.m_armSubsystem.setPos(m_deg + m_adjustment);
    //   }
  
    // }
    // Robot.m_armSubsystem.setStall(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean reached = false;
    if(m_final){
      // System.out.println(String.format("Cur Angle: %f, Set Point: %f", Robot.m_armSubsystem.getAngleFinal(), m_deg));
      reached = Math.abs(Robot.m_armSubsystem.getAngleFinal() - (m_deg + m_adjustment)) <= k_error;
      Logger.Log("MoveArmPos", 2, String.format("Angle: %f, ideal pos: %f", Robot.m_armSubsystem.getAngleFinal(), m_deg + m_adjustment));
    }else{
      reached = Math.abs(Robot.m_armSubsystem.getAngle() - (m_deg + m_adjustment)) <= k_error;
      Logger.Log("MoveArmPos", 2, String.format("Angle: %f, ideal pos: %f", Robot.m_armSubsystem.getAngle(), m_deg + m_adjustment));
    }

    // SmartDashboard.putBoolean("Reached", reached);
    return reached;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // System.out.println("Arm Ended");
    // Robot.m_armSubsystem.stop();

    Robot.m_armSubsystem.setStall(true);

    Logger.Log("MoveArmPosCommand", 3, "End");
    // System.out.println("Move Arm Pos End");
  }
}
