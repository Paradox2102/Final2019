/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.grabber.OpenGrabberCommand;
import frc.robot.commands.grabberWheels.OuttakeBurpCommand;
import frc.robot.commands.grabberWheels.OuttakeGrabberCommand;
import frc.robot.commands.intakeWheels.OuttakeCargoCommand;

public class OuttakeAllCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public OuttakeAllCommand() {
    addParallel(new OuttakeBurpCommand());
    addSequential(new OuttakeCargoCommand(1));
  }
}
