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
import frc.robot.commands.climber.ReleasePistonsDoubleCommand;
import frc.robot.commands.intake.MoveIntakePosCommand;

public class ReleasePistonsLevel2Command extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ReleasePistonsLevel2Command() {
    addSequential(new MoveIntakePosCommand(-30, RobotMap.k_finalRobot));
    addSequential(new WaitCommand(0.25));
    addSequential(new ReleasePistonsDoubleCommand());
  }
}
