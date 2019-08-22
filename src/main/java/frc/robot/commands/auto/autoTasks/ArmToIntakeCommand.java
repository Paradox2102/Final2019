/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.MoveArmToIntakeCommand;
import frc.robot.commands.grabber.ConditionalOpenGrabberCommand;

public class ArmToIntakeCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ArmToIntakeCommand() {
    // addSequential(new ConditionalOpenGrabberCommand());
    // addSequential(new WaitCommand(1));
    addSequential(new MoveArmToIntakeCommand());
  }
}
