/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.pathfinder.Pathfinder;
import frc.pathfinder.Pathfinder.Path;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Robot;
import frc.robot.PurePursuit.PathConfig;

/**
 * Add your docs here.
 */
public class PrintPathCommand extends InstantCommand {
  Path m_path;
  public PrintPathCommand(Waypoint[] points, PathConfig config) {
    super();
    requires(Robot.m_driveSubsystem);

    m_path = Pathfinder.computePath(points, config.m_points, config.m_dt, config.m_vel, config.m_accel, config.m_jerk, config.m_wheelBase);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.printPath(m_path);
  }

}
