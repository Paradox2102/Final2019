/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.Condition;
import frc.robot.Robot;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.elevator.ConditionalElevatorMoveCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;

public class ConditionalOpenGrabberCommand extends CommandGroup {
  public class MoveElevatorCondition implements Condition{
    public boolean condition(){
      return Robot.m_elevatorSubsystem.getPosRelativeHome() < 100 && Robot.m_armSubsystem.getAngle() < 10;
    }
  }
  /**
   * Add your docs here.
   */
  public ConditionalOpenGrabberCommand() {
    addParallel(new ConditionalElevatorMoveCommand(new MoveElevatorCondition(), new MoveElevatorPosCommand(ElevatorLevel.FeederPlaceHeight), new DoNothingCommand()));
    addSequential(new OpenGrabberCommand());
  }
}
