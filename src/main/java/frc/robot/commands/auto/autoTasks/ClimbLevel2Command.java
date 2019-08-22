/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.commands.LED.SetClimbLEDCommand;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.climber.ActivatePistonsCommand;
import frc.robot.commands.climber.Level2ActivatePistonsCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.grabber.CloseGrabberCommand;
import frc.robot.commands.grabber.OpenGrabberCommand;
import frc.robot.commands.intake.IntakeClimbCommand;
import frc.robot.commands.intake.MoveIntakePosCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbLevel2Command extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimbLevel2Command() {
    addSequential(new SetClimbLEDCommand(true));
    addSequential(new SetClimbLEDCommand(true));
    addSequential(new CloseGrabberCommand());
    addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    addSequential(new OpenGrabberCommand());
    addSequential(new MoveElevatorPosCommand(ElevatorLevel.ClimbHeight));
    addSequential(new MoveIntakePosCommand(RobotMap.k_intakeClimbLevel2Pos, RobotMap.k_finalRobot));
    addSequential(new ElevatorHomeCommand());
    addSequential(new CloseGrabberCommand());
    addParallel(new IntakeClimbCommand(0.75, -5));
    addSequential(new CommandGroup() {
      {
        addSequential(new WaitCommand(1));
        addSequential(new Level2ActivatePistonsCommand());
      }
    });
  }
}
