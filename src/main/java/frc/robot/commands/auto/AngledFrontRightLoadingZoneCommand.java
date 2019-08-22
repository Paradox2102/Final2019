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

public class AngledFrontRightLoadingZoneCommand extends CommandGroup {
  private final Waypoint[] angledFrontRightCargoShipToRightLoadingZone = {
    new Waypoint(1.6, 15.3, Math.toRadians(-65), 1, 1, 0),
    new Waypoint(6, 13, Math.toRadians(-20), 4, 5, 0),
    new Waypoint(11.36, 4, Math.toRadians(-90), 1),
  };//moved over point 0.5ft
  public AngledFrontRightLoadingZoneCommand() {
    addSequential(new CreatePathCommand(angledFrontRightCargoShipToRightLoadingZone, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, false));

  }
}
