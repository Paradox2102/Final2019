/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.Condition;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.elevator.ConditionalElevatorMoveCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robotCore.Logger;

public class TeleopCargoShipPlacement extends CommandGroup {
  class AvoidIntakeCondition implements Condition{
    public boolean condition(){
      return (Robot.m_intakeSubsystem.getLimit() || Robot.m_intakeSubsystem.getAngle() > 90);
    }
  }
  
  class 
  AvoidArmCondition implements Condition{
    public boolean condition(){
      if(RobotMap.k_finalRobot){
        Logger.Log("AvoidArmCondition", 2, String.format("Elevator Pos: %d, Arm Angle: %f", Robot.m_elevatorSubsystem.getPosRelativeHome(), Robot.m_armSubsystem.getAngleFinal()));
        return (Robot.m_elevatorSubsystem.getPosRelativeHome() < RobotMap.k_elevatorIntakeHeight + 500 && Robot.m_armSubsystem.getAngleFinal() < 0);
      }else{
        Logger.Log("AvoidArmCondition", 2, String.format("Elevator Pos: %d, Arm Angle: %f", Robot.m_elevatorSubsystem.getPosRelativeHome(), Robot.m_armSubsystem.getAngle()));
        return (Robot.m_elevatorSubsystem.getPosRelativeHome() < RobotMap.k_elevatorIntakeHeight + 500 && Robot.m_armSubsystem.getAngle() < 0);
      }
    }
  }

  class ArmClear implements Condition{
    private double m_clearing;
    public ArmClear(double clearing){
      m_clearing = clearing;
    }
    public boolean condition(){
      if(RobotMap.k_finalRobot){
        return Robot.m_armSubsystem.getAngleFinal() > m_clearing;
      }else{
        return Robot.m_armSubsystem.getAngle() > m_clearing;
      }
    }
  }
  public TeleopCargoShipPlacement(boolean front) {
    addSequential(new ConditionalElevatorMoveCommand(new AvoidIntakeCondition(), new MoveElevatorPosCommand(ElevatorLevel.CargoAvoidanceLevel),
      new ConditionalElevatorMoveCommand(new AvoidArmCondition(), new MoveElevatorPosCommand(ElevatorLevel.ArmAvoid), new DoNothingCommand())));
    addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    addSequential(new MoveElevatorPosCommand(ElevatorLevel.CargoShip));
    if(front){
      addSequential(new MoveArmPosCommand(ArmPos.CargoForward, RobotMap.k_finalRobot));
    }else{
      addSequential(new MoveArmPosCommand(ArmPos.CargoBackward, RobotMap.k_finalRobot));
    }
  }
}
