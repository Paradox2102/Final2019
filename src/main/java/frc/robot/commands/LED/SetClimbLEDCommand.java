/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Add your docs here.
 */
public class SetClimbLEDCommand extends InstantCommand {
  /**
   * Add your docs here.
   */
  LEDSubsystem m_ledSubsysted;
  boolean m_climbing;
  public SetClimbLEDCommand(boolean climbing) {
    super();
    m_climbing = climbing;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_LEDSubsystem1.setClimbing(m_climbing);
    Robot.m_LEDSubsystem2.setClimbing(m_climbing);
  }

}
