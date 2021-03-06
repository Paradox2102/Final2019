/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.grabberWheels;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.grabber.OpenGrabberCommand;

public class OuttakeBurpCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public OuttakeBurpCommand() {
    addParallel(new OuttakeGrabberCommand());
    addSequential(new OpenGrabberCommand());
  }

  public OuttakeBurpCommand(double time) {
    addParallel(new OuttakeGrabberCommand(time));
    addSequential(new OpenGrabberCommand());
  }
}
