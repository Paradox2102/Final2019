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

public class AngledRightLoadingZoneRightSideCargoShipCommand extends CommandGroup {
  private final Waypoint[] angledRightLoadingZoneSideCargoShip = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 1.5, 1, 0),
    new Waypoint(7.5, 18, Math.toRadians(155), 3.3),
  };//moved over point 0.5ft
  public AngledRightLoadingZoneRightSideCargoShipCommand() {
    addSequential(new CreatePathCommand(angledRightLoadingZoneSideCargoShip, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFastest, true, true, false, false, false));

  }
}
