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

public class RightFrontRightCargoShipCommand extends CommandGroup {
  public RightFrontRightCargoShipCommand() {
    // addSequential(new CreatePathCommand(new Waypoint[] {
    //   new Waypoint(5.4 - RobotMap.k_chassisWidth/2.0, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 4, 6),
    //   new Waypoint(0.4, 18.35 - RobotMap.k_chassisLength/2.0 - RobotMap.k_frontCSError + 0.5, Math.toRadians(90), 0)
    // }, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, true));

    addSequential(new CreatePathCommand(new Waypoint[] {
      new Waypoint(5.4 - RobotMap.k_chassisWidth/2.0, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 3, 6),
      new Waypoint(2.75, 14, Math.toRadians(115), 3.3)
    }, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
  }
}
