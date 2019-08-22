/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Navigator;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Navigator.CameraDirection;
import frc.robot.Navigator.FieldSide;
import frc.robot.Navigator.RocketSide;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.elevator.MoveElevatorPosCommand.RobotMode;

public class PlatformRocketCommand extends CommandGroup {
  private final Waypoint[] rightSideToRightCloseRocketPath = {
    // new Waypoint(5.4 - RobotMap.k_chassisWidth/2.0, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 4, 3, 6),
    // new Waypoint(8.7, 13, Math.toRadians(60), 0, 0, 3.3)
    new Waypoint(5.4 - RobotMap.k_chassisWidth/2.0, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 3, 6),
    new Waypoint(8.2, 12.13, Math.toRadians(60), 0, 0, 3.3)
  };
  private final Waypoint[] leftSideToLeftCloseRocketPath = {
    // new Waypoint(5.4 - RobotMap.k_chassisWidth/2.0, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 4, 3, 6),
    // new Waypoint(8.7, 13, Math.toRadians(60), 0, 0, 3.3)
    new Waypoint(-(5.4 - RobotMap.k_chassisWidth/2.0), 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 3, 6),
    new Waypoint(-8.2, 12.13, Math.toRadians(120), 0, 0, 2)
  };
  public PlatformRocketCommand(FieldSide fieldSide, RocketSide rocketSide, ElevatorLevel level) {
    if(fieldSide == FieldSide.Right){
      if(rocketSide == RocketSide.Close){
        addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
        addSequential(new CreatePathCommand(rightSideToRightCloseRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
        addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, level, RobotMode.Hatch));
        addParallel(new ArmForwardCommand(false));
        addSequential(new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level1, true));
        addSequential(new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level2));
        // addSequential(new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level1, true));
      }
    }else{
      if(rocketSide == RocketSide.Close){
        addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
        addSequential(new CreatePathCommand(leftSideToLeftCloseRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
        addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, level, RobotMode.Hatch, 1.25));
        addSequential(new RocketLoadingZoneCommand(FieldSide.Left, RocketSide.Close, ElevatorLevel.Level1, true));
        addSequential(new LoadingZoneRocketCommand(FieldSide.Left, RocketSide.Close, ElevatorLevel.Level2));
        // addSequential(new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level1, true));
      }
    }
  }
}
