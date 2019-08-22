/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;

public class ConditionalHatchModeCommand extends ConditionalCommand {
  public ConditionalHatchModeCommand(final Command leftCmd, final Command rightCmd) {
    super(leftCmd, rightCmd);
}

protected boolean condition() {
  return Robot.m_oi.hatchMode();
}
}
