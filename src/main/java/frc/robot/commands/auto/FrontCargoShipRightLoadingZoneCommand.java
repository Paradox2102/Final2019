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
import frc.robot.Navigator.CameraDirection;
import frc.robot.Navigator.FieldSide;

public class FrontCargoShipRightLoadingZoneCommand extends CommandGroup {
  private final Waypoint[] frontLeftCargoShipToRightLoadingZone = {
    new Waypoint(-0.9, 18.35 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90)),
    new Waypoint(6, 12, Math.toRadians(0), 4, 5, 0),
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0 + RobotMap.k_frontCSError, Math.toRadians(-90), 0),
  };
  // private final Waypoint[] frontRightCargoShipToRightLoadingZone = {
  //   // new Waypoint(0.9, 18.35 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90)),
  //   // new Waypoint(6, 13, Math.toRadians(0), 4, 5, 0),
  //   // new Waypoint(11.36, RobotMap.k_chassisLength/2.0 + RobotMap.k_frontCSError, Math.toRadians(-90), 3.3),
  //   new Waypoint(0.9, 18.35 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90), 2, 2, 0),
  //   new Waypoint(6, 13, Math.toRadians(-20), 4, 5, 0),
  //   new Waypoint(11.36, 6, Math.toRadians(-90), 3.3),
  // };
  private final Waypoint[] frontRightCargoShipToRightLoadingZone = {
    new Waypoint(0.9, 18.35 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90), 2, 2, 0),
    new Waypoint(6, 13, Math.toRadians(-20), 4, 5, 0),
    new Waypoint(11.36, 4, Math.toRadians(-90)),
  };//moved over point 0.5ft

  public FrontCargoShipRightLoadingZoneCommand(FieldSide frontCargoShipSide, boolean cameraCorrection) {
    if(frontCargoShipSide == FieldSide.Left){
      addSequential(new CreatePathCommand(frontLeftCargoShipToRightLoadingZone, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false));//, cameraCorrection, CameraDirection.Front));
    } else{
      addSequential(new CreatePathCommand(frontRightCargoShipToRightLoadingZone, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, false));
    }
  }
}
