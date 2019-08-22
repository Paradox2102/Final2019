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

public class LoadingZoneFrontCargoShipCommand extends CommandGroup {
  private final Waypoint[] rightLoadingZoneToFrontRightCargoShipPath = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0 + RobotMap.k_frontCSError, Math.toRadians(90)),
    new Waypoint(6, 10, Math.toRadians(180), 5, 5, 4),
    new Waypoint(0.9, 18.35 - RobotMap.k_chassisLength/2.0 - RobotMap.k_frontCSError, Math.toRadians(90))
  };

  // private final Waypoint[] rightLoadingZoneToFrontLeftCargoShipPath = {
  //   new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90)),
  //   new Waypoint(6, 9.5, Math.toRadians(180), 2, 5, 0),
  //   new Waypoint(-0.9, 14, Math.toRadians(90), 3.3)
  //   // new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 2, 0),
  //   // new Waypoint(6.5, 8.5, Math.toRadians(155), 2, 5, 0),
  //   // new Waypoint(-0.9, 14.5, Math.toRadians(90), 3.3)
  // };

  final Waypoint[] rightLoadingZoneToFrontLeftCargoShipPath = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90),6.5,4,10),
    // new Waypoint(-6.5, 9.5, Math.toRadians(20), 2, 4, 0),
    new Waypoint(2.5, 13.5, Math.toRadians(125), 2)
    // new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 2, 0),
    // new Waypoint(6.5, 8.5, Math.toRadians(155), 2, 5, 0),
    // new Waypoint(-0.9, 14.5, Math.toRadians(90), 3.3)
  };

  private final Waypoint[] leftLoadingZoneToFrontLeftCargoShipPath = {
    new Waypoint(-11.36, RobotMap.k_chassisLength/2.0 + RobotMap.k_frontCSError, Math.toRadians(90)),
    new Waypoint(-6, 10, Math.toRadians(0), 5, 5, 4),
    new Waypoint(-0.9, 18.35 - RobotMap.k_chassisLength/2.0 - RobotMap.k_frontCSError, Math.toRadians(90))
  };

  // private final Waypoint[] leftLoadingZoneToFrontRightCargoShipPath = {
  //   new Waypoint(-11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90)),
  //   new Waypoint(-6, 9.5, Math.toRadians(0), 2, 4, 0),
  //   new Waypoint(0.9, 13, Math.toRadians(90), 0)
  //   // new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 2, 0),
  //   // new Waypoint(6.5, 8.5, Math.toRadians(155), 2, 5, 0),
  //   // new Waypoint(-0.9, 14.5, Math.toRadians(90), 3.3)
  // };

  final Waypoint[] leftLoadingZoneToFrontRightCargoShipPath = {
    new Waypoint(-11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90),6.5,3,10),
    // new Waypoint(-6.5, 9.5, Math.toRadians(20), 2, 4, 0),
    new Waypoint(-2.5 + RobotMap.k_leftLoadingZoneKludge, 13.5, Math.toRadians(55), 3.3)
    // new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 2, 0),
    // new Waypoint(6.5, 8.5, Math.toRadians(155), 2, 5, 0),
    // new Waypoint(-0.9, 14.5, Math.toRadians(90), 3.3)
  };

  public LoadingZoneFrontCargoShipCommand(FieldSide loadingSide, boolean cameraCorrection) {
    if(loadingSide == FieldSide.Right){
      addSequential(new CreatePathCommand(rightLoadingZoneToFrontRightCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, cameraCorrection, CameraDirection.Front));
    } else{
      addSequential(new CreatePathCommand(leftLoadingZoneToFrontLeftCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, cameraCorrection, CameraDirection.Front));
    }
  }

  public LoadingZoneFrontCargoShipCommand(FieldSide loadingSide, FieldSide cargoShipPort, boolean cameraCorrection) {
    if(loadingSide == FieldSide.Right){
      if(cargoShipPort == FieldSide.Right){
        addSequential(new CreatePathCommand(rightLoadingZoneToFrontRightCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, cameraCorrection, CameraDirection.Front));
      }else{
        addSequential(new CreatePathCommand(rightLoadingZoneToFrontLeftCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, cameraCorrection, false));
      }
    } else{
      if(cargoShipPort == FieldSide.Right){
        addSequential(new CreatePathCommand(leftLoadingZoneToFrontRightCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, cameraCorrection, false));
      }else{
        addSequential(new CreatePathCommand(leftLoadingZoneToFrontLeftCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, cameraCorrection, CameraDirection.Front));
      }
    }
  }
}