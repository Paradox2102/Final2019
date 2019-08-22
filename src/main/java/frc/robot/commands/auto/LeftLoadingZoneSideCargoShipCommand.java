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
import frc.robot.Navigator.CargoSide;


public class LeftLoadingZoneSideCargoShipCommand extends CommandGroup {
  private final Waypoint[] leftLoadingZoneToSideCloseCargoShipPath = {
    new Waypoint(-11.36, RobotMap.k_frontCSError + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 2, 0),
    new Waypoint(-9.25, 13, Math.toRadians(90), 4, 5, 4),
    new Waypoint(-2.4 - RobotMap.k_sideCSError - RobotMap.k_chassisLength/2.0, 21.5, Math.toRadians(0))
  };
  private final Waypoint[] leftLoadingZoneToSideMiddleCargoShipPath = {
    new Waypoint(-11.36, RobotMap.k_frontCSError + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 3, 0),
    new Waypoint(-9.25, 13, Math.toRadians(90), 5, 5, 4),
    new Waypoint(-2.4 - RobotMap.k_sideCSError - RobotMap.k_chassisLength/2.0, 23.3, Math.toRadians(0))
  };
  private final Waypoint[] leftLoadingZoneToSideFarCargoShipPath = {
    new Waypoint(-11.36, RobotMap.k_frontCSError + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 3, 0),
    new Waypoint(-9.25, 16, Math.toRadians(90), 5, 5, 4),
    new Waypoint(-2.4 - RobotMap.k_sideCSError - RobotMap.k_chassisLength/2.0, 25.13, Math.toRadians(0))
  };
  public LeftLoadingZoneSideCargoShipCommand(CargoSide cargoSide) {
    if(cargoSide == CargoSide.Close){
      addSequential(new CreatePathCommand(leftLoadingZoneToSideCloseCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, CameraDirection.Front));
    } else if(cargoSide == CargoSide.Middle){
      addSequential(new CreatePathCommand(leftLoadingZoneToSideMiddleCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, CameraDirection.Front));
    } else{
      addSequential(new CreatePathCommand(leftLoadingZoneToSideFarCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, CameraDirection.Front));
    }
  }
}
