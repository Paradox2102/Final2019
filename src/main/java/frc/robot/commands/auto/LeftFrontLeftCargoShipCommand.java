/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class LeftFrontLeftCargoShipCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftFrontLeftCargoShipCommand() {
    addSequential(new CreatePathCommand(new Waypoint[] {
      new Waypoint(-(5.4 - RobotMap.k_chassisWidth/2.0), 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 3, 6),
      new Waypoint(-2.75, 14, Math.toRadians(65), 3.3)
    }, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
  }
}
