/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.lib.Condition;
import frc.lib.ConditionCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.AvoidIntakeCommand;
import frc.robot.commands.elevator.ConditionalElevatorMoveCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.grabber.CloseGrabberCommand;
import frc.robot.commands.intake.MoveIntakePosCommand;
import frc.robot.commands.intake.WaitForIntakeCommand;
import frc.robotCore.Logger;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.WaitForArmCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.DoNothingCommand;

public class IntakeVerticalCommand extends CommandGroup {
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
  /**
   * Add your docs here.
   */
  public IntakeVerticalCommand() {
    
    addSequential(new ConditionalElevatorMoveCommand(new AvoidIntakeCondition(), new MoveElevatorPosCommand(ElevatorLevel.CargoAvoidanceLevel),
      new ConditionalElevatorMoveCommand(new AvoidArmCondition(), new MoveElevatorPosCommand(ElevatorLevel.ArmAvoid), new DoNothingCommand())));
    addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    addSequential(new ConditionCommand(new ArmClear(0), new DoNothingCommand(), new WaitForArmCommand(0, 5)));
    addSequential(new CloseGrabberCommand());
    addParallel(new CommandGroup(){
      {
        addSequential(new WaitForIntakeCommand(75, 8, 2));
        addSequential(new ElevatorHomeCommand());
      }
    });
    addSequential(new MoveIntakePosCommand(75, RobotMap.k_finalRobot));
  }
}
