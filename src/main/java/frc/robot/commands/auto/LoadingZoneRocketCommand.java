/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.Condition;
import frc.lib.ConditionCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Navigator;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Navigator.FieldSide;
import frc.robot.Navigator.RobotTarget;
import frc.robot.Navigator.RocketSide;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.elevator.MoveElevatorPosCommand.RobotMode;
import frc.robotCore.Logger;

public class LoadingZoneRocketCommand extends CommandGroup {
  
  private final Waypoint[] rightLoadingZoneToCloseRightRocketPath = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 5, 0),
    new Waypoint(9.21, 13, Math.toRadians(60), 0, 0, 3.3)
  };

  private final Waypoint[] leftLoadingZoneToCloseLeftRocketPath = {
    new Waypoint(-11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 5, 0),
    new Waypoint(-9.21, 13, Math.toRadians(120), 0, 0, 2.2)
  };

  private final Waypoint[] rightLoadingZoneToMiddleRocketPath = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 3, 9),
    new Waypoint(6.25, 19.1, Math.toRadians(180))
  };

  private final Waypoint[] rightLoadingZoneToFarRocketPath = {
    new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 5, 12),
    new Waypoint(9.1, 13.5, Math.toRadians(100), 4, 2.5, 12),
    new Waypoint(9.3, 24.6, Math.toRadians(120))
  };

  public class DriveForward implements Condition{
    FieldSide m_fieldSide;

    public DriveForward(FieldSide fieldSide){
      m_fieldSide = fieldSide;
    }

    public boolean condition(){
      RobotTarget target;
      if(FieldSide.Right == m_fieldSide){
        target = Navigator.m_targets[15];
      }else{
        target = Navigator.m_targets[14];
      }
      double angle = Math.abs(Navigator.normalizeAngleDeg(Math.toDegrees(target.m_angle) - Robot.m_driveSubsystem.getAngle()));
      Logger.Log("Loading Zone Rocket", 3, String.format("Target Angle: %f, Cur Angle: %f, Condition: %b", target.m_angle, Robot.m_driveSubsystem.getAngle(), angle > 90));
      return angle > 90;
    }
  }

  public LoadingZoneRocketCommand(FieldSide fieldSide, RocketSide rocketSide, ElevatorLevel level) {
    if(fieldSide == FieldSide.Right){
      if(rocketSide == RocketSide.Close){
        addSequential(new ConditionCommand(new DriveForward(fieldSide), new CommandGroup(){
          {
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new CreatePathCommand(rightLoadingZoneToCloseRightRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, false));
            addSequential(new DriveDropOffBackUpHatchCommand(false, false, false, level, RobotMode.Hatch));
          }
        }, new CommandGroup(){
          {
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new CreatePathCommand(rightLoadingZoneToCloseRightRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
            addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, level, RobotMode.Hatch));
          }
        }));
      }else if(rocketSide == RocketSide.Far){
        addSequential(new ConditionCommand(new DriveForward(fieldSide), new CommandGroup(){
          {
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new CreatePathCommand(rightLoadingZoneToFarRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false));
            addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, level, RobotMode.Hatch));
          }
        }, new CommandGroup(){
          {
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new CreatePathCommand(rightLoadingZoneToFarRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false));
            addSequential(new DriveDropOffBackUpHatchCommand(false, false, false, level, RobotMode.Hatch));
          }
        }));
      }else if(rocketSide == RocketSide.Middle){
        //for cargo to release, takes in driver input
        addSequential(new ConditionCommand(new DriveForward(fieldSide), new CommandGroup(){
          {
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new CreatePathCommand(rightLoadingZoneToMiddleRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false));
            addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, level, RobotMode.Cargo));
          }
        }, new CommandGroup(){
          {
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new CreatePathCommand(rightLoadingZoneToMiddleRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false));
            addSequential(new DriveDropOffBackUpHatchCommand(false, false, false, level, RobotMode.Cargo));
          }
        }));
      }
    }else{
      if(rocketSide == RocketSide.Close){
        addSequential(new ConditionCommand(new DriveForward(fieldSide), new CommandGroup(){
          {
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new CreatePathCommand(leftLoadingZoneToCloseLeftRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, false));
            addSequential(new DriveDropOffBackUpHatchCommand(false, false, false, level, RobotMode.Hatch));
          }
        }, new CommandGroup(){
          {
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new CreatePathCommand(leftLoadingZoneToCloseLeftRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
            addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, level, RobotMode.Hatch));
          }
        }));
      }
    }
  }
}
