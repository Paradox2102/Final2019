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

public class FrontCargoShipLeftLoadingZoneCommand extends CommandGroup {
  private final Waypoint[] frontLeftCargoShipToLeftLoadingZone = {
    new Waypoint(-0.9, 18.35 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90), 2, 2, 0),
    new Waypoint(-6, 13, Math.toRadians(-160), 4, 5, 0),
    new Waypoint(-11.36 - RobotMap.k_leftLoadingZoneKludge, 4, Math.toRadians(-90)),
  };//moved over point 0.5ft
  private final Waypoint[] frontRightCargoShipToLeftLoadingZone = {
    new Waypoint(0.9, 18.35 - RobotMap.k_chassisLength/2.0, Math.toRadians(90)),
    new Waypoint(-6, 10, Math.toRadians(180), 5, 5, 4),
    new Waypoint(-11.36, RobotMap.k_chassisLength/2.0 + RobotMap.k_frontCSError, Math.toRadians(90)),
  };
  
  public FrontCargoShipLeftLoadingZoneCommand(FieldSide frontCargoShipSide, boolean cameraCorrection) {
    if(frontCargoShipSide == FieldSide.Left){
      addSequential(new CreatePathCommand(frontLeftCargoShipToLeftLoadingZone, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, cameraCorrection, true));
    } else{
      addSequential(new CreatePathCommand(frontRightCargoShipToLeftLoadingZone, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, cameraCorrection, CameraDirection.Front));
    }
  }
}
