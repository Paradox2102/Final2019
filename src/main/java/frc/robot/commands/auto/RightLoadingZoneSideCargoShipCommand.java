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

public class RightLoadingZoneSideCargoShipCommand extends CommandGroup {
  private final Waypoint[] rightLoadingZoneToSideCloseCargoShipPath = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 2, 0),
    new Waypoint(8.5, 13, Math.toRadians(90), 3, 3, 4),
    new Waypoint(2.4 + RobotMap.k_chassisLength/2.0 + RobotMap.k_sideCSError, 22, Math.toRadians(180))
  };
  private final Waypoint[] rightLoadingZoneToSideMiddleCargoShipPath = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 3, 0),
    new Waypoint(8.5, 13, Math.toRadians(90), 3, 3, 4),
    new Waypoint(2.4 + RobotMap.k_chassisLength/2.0 + RobotMap.k_sideCSError, 23.3, Math.toRadians(180))
  };
  private final Waypoint[] rightLoadingZoneToSideFarCargoShipPath = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 3, 0),
    new Waypoint(9.25, 16, Math.toRadians(90), 5, 5, 4),
    new Waypoint(2.4 + RobotMap.k_chassisLength/2.0 + RobotMap.k_sideCSError, 25.13, Math.toRadians(180))
  };
  public RightLoadingZoneSideCargoShipCommand(CargoSide cargoSide) {
    if(cargoSide == CargoSide.Close){
      addSequential(new CreatePathCommand(rightLoadingZoneToSideCloseCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false));
    } else if(cargoSide == CargoSide.Middle){
      addSequential(new CreatePathCommand(rightLoadingZoneToSideMiddleCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false));
    } else{
      addSequential(new CreatePathCommand(rightLoadingZoneToSideFarCargoShipPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false));
    }
  }
}
