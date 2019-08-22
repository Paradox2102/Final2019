/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Navigator.CameraDirection;
import frc.robot.Navigator.FieldSide;
import frc.pathfinder.Pathfinder.Waypoint;

public class CenterFrontCargoShipCommand extends CommandGroup {
  private final Waypoint[] centerToFrontLeftCargoShipPath = {
    new Waypoint(-0.9, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 0, 0, 6.5),
    new Waypoint(-0.9, 11.5,  Math.toRadians(90), 3.3)
  };

  // private final Waypoint[] centerToFrontRightCargoShipPath = {
  //   new Waypoint(2 - RobotMap.k_chassisLength/2.0, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(-90)),
  //   new Waypoint(0.9, 18.35 - RobotMap.k_chassisLength/2.0 - RobotMap.k_frontCSError, Math.toRadians(-90)),
  // };

  private final Waypoint[] centerToFrontRightCargoShipPath = {
    new Waypoint(.9, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 0, 0, 6.5),
    new Waypoint(.9, 11.5, Math.toRadians(90), 0, 0, 2)
  };

  public CenterFrontCargoShipCommand(FieldSide fieldSide, boolean cameraCorrection) {
    if(fieldSide == FieldSide.Left){
      addSequential(new CreatePathCommand(centerToFrontLeftCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, cameraCorrection, CameraDirection.Front, false));
    } else{
      addSequential(new CreatePathCommand(centerToFrontRightCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
    }
  }
}