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

public class AngledFrontLeftLoadingZoneCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  private final Waypoint[] angledFrontLeftCargoShipToLeftLoadingZone = {
    new Waypoint(-1.6, 15.3, Math.toRadians(-115), 1, 1, 0),
    new Waypoint(-6, 13, Math.toRadians(-160), 4, 5, 0),
    new Waypoint(-11.36 - RobotMap.k_leftLoadingZoneKludge, 4, Math.toRadians(-90), 1),
  };//moved over point 0.5ft
  public AngledFrontLeftLoadingZoneCommand() {
    addSequential(new CreatePathCommand(angledFrontLeftCargoShipToLeftLoadingZone, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, false));
  }
}
