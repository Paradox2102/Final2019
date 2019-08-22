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
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.elevator.ConditionalElevatorMoveCommand;
import frc.robot.commands.elevator.ElevatorLowPlacementCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;

class ElevatorGrab implements Condition{
  public boolean condition(){
    return (!Robot.m_armSubsystem.hasHatch() && Robot.m_elevatorSubsystem.getPosRelativeHome() < 500);
  }
}

class ElevatorPlace implements Condition{
  public boolean condition(){
    return (Robot.m_armSubsystem.hasHatch() && Robot.m_elevatorSubsystem.getPosRelativeHome() < 500);
  }
}

class HasHatch implements Condition{
  public boolean condition(){
    return Robot.m_armSubsystem.hasHatch();
  }
}

public class ArmForwardCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ArmForwardCommand(boolean driverInput) {
    addSequential(new ConditionalElevatorMoveCommand(new ElevatorGrab(), new MoveElevatorPosCommand(ElevatorLevel.FeederGrabHeight),
                  new ConditionCommand(new ElevatorPlace(), new MoveElevatorPosCommand(ElevatorLevel.FeederPlaceHeight), new DoNothingCommand())));
    // addSequential(new MoveElevatorPosCommand(ElevatorLevel.FeederPlaceHeight));\
    addSequential(new MoveArmPosCommand(ArmPos.Forward, RobotMap.k_finalRobot, driverInput));
    // addSequential(new ConditionCommand(new HasHatch(), new MoveArmPosCommand(ArmPos.Forward, RobotMap.k_finalRobot, driverInput), new MoveArmPosCommand(-8, RobotMap.k_finalRobot)));
  }

  public ArmForwardCommand(boolean driverInput, ArmPos armPos) {
    addSequential(new ConditionalElevatorMoveCommand(new ElevatorGrab(), new MoveElevatorPosCommand(ElevatorLevel.FeederPlaceHeight), new DoNothingCommand()));
    addSequential(new MoveArmPosCommand(armPos, RobotMap.k_finalRobot, driverInput));
  }
}
