/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.calibrate;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.CreatePathCommand;
import frc.robot.commands.drive.SetBrakeModeCommand;
import frc.robot.commands.drive.SetCoastModeCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Navigator.CameraDirection;

public class CameraErrorTestCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CameraErrorTestCommand() {
    addSequential(new SetBrakeModeCommand());
    addSequential(new CreatePathCommand(new Waypoint[] {
			new Waypoint(0, 0 + RobotMap.k_chassisLength/2.0, Math.toRadians(90)),
      new Waypoint(0, 13 - RobotMap.k_chassisLength/2.0, Math.toRadians(90)),
    }, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, true, CameraDirection.Front));
    addSequential(new SetCoastModeCommand());
  }
}
