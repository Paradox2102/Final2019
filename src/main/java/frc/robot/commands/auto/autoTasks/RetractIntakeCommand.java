/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.Condition;
import frc.lib.ConditionCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.WaitForArmCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.WaitForElevatorCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.intake.IntakeHomeCommand;

public class RetractIntakeCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  class ElevatorClear implements Condition{
    double m_pos;
    public ElevatorClear(double pos){
      m_pos = pos;
    }
    public boolean condition(){
      return Robot.m_elevatorSubsystem.getPosRelativeHome() > m_pos;
    }
  }
  public RetractIntakeCommand(double intakePower) {
    addParallel(new ConditionCommand(new ElevatorClear(4200), new DoNothingCommand(), new CommandGroup(){
      {
        addSequential(new MoveElevatorPosCommand(ElevatorLevel.CargoAvoidanceLevel));
      }
    }));
    addSequential(new ConditionCommand(new ElevatorClear(4200), new DoNothingCommand(), new WaitForElevatorCommand(4200)));
    addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    addSequential(new ConditionCommand(new ElevatorClear(4200), new DoNothingCommand(), new WaitForElevatorCommand(RobotMap.k_elevatorIntakeAvoidHeight - 100)));
    addSequential(new IntakeHomeCommand(intakePower));
    addSequential(new ElevatorHomeCommand());
  }
}
