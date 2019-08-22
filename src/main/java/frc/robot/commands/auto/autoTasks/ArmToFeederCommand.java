/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.grabber.OpenGrabberCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToFeederCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ArmToFeederCommand(boolean forward) {
    addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    addParallel(new ElevatorHomeCommand());
    addSequential(new OpenGrabberCommand());
    if(forward){
      addSequential(new MoveArmPosCommand(ArmPos.FeederForward, RobotMap.k_finalRobot));
    }else{
      addSequential(new MoveArmPosCommand(ArmPos.FeederBackward, RobotMap.k_finalRobot));
    }
  }
}
