/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
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
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.elevator.MoveElevatorPosCommand.RobotMode;
import frc.robot.commands.grabber.CloseGrabberCommand;
import frc.robotCore.Logger;

public class RocketLoadingZoneCommand extends CommandGroup {
  private final Waypoint[] closeRightRocketToRightLoadingZonePath = {
    new Waypoint(11.17, 16.4, Math.toRadians(-120), 2, 4, 0),
    new Waypoint(11.36, 8 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90), 0, 0, 1)
  };

  private final Waypoint[] closeLeftRocketToLeftLoadingZonePath = {
    new Waypoint(-11.17, 16.4, Math.toRadians(-60), 2, 4, 0),
    new Waypoint(-11.36, 8 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90), 0, 0, 1)
  };

  private final Waypoint[] middleRocketToRightLoadingZoneV2Path = { //curved
    new Waypoint(11.1, 19.14, Math.toRadians(180), 6, 4, 0),//abs 11.95,24.2
    new Waypoint(11.36, 6 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90))
  };

  private final Waypoint[] farRocketToRightLoadingZoneV1P1Path = { //two paths, goes forward and backward
    new Waypoint(11.2, 21.8, Math.toRadians(120), 1, 1, 3),//abs 11.95,24.2
    new Waypoint(9.3, 24.6, Math.toRadians(100)),
  };

  private final Waypoint[] farRocketToRightLoadingZoneV1P2Path = {
    new Waypoint(9.3, 24.6, Math.toRadians(-100), 5, 5, 9),
    new Waypoint(10.86, 6 - RobotMap.k_chassisLength/2.0, Math.toRadians(-90))
  };

  public class DriveBackwards implements Condition{
    FieldSide m_fieldSide;
    RocketSide m_rocketSide;

    public DriveBackwards(FieldSide fieldSide, RocketSide rocketSide){
      m_fieldSide = fieldSide;
      m_rocketSide = rocketSide;
    }

    public boolean condition(){
      RobotTarget target;
      if(m_fieldSide == FieldSide.Right){
        if(m_rocketSide == RocketSide.Close){
          target = Navigator.m_targets[11];
        }else if(m_rocketSide == RocketSide.Far){
          target = Navigator.m_targets[13];
        }else{
          target = Navigator.m_targets[12];
        }
      }else{
        if(m_rocketSide == RocketSide.Close){
          target = Navigator.m_targets[8];
        }else if(m_rocketSide == RocketSide.Far){
          target = Navigator.m_targets[10];
        }else{
          target = Navigator.m_targets[9];
        }
      }

      double angle = Math.abs(Navigator.normalizeAngleDeg(Math.toDegrees(target.m_angle) - Robot.m_driveSubsystem.getAngle()));
      Logger.Log("Rocket Loading Zone Drive Backwards", 3, String.format("Condition: %b, angle: %f, target Angle: %f, gyro angle: %f", !(angle > 90 || angle < -90), angle, Math.toDegrees(target.m_angle), Robot.m_driveSubsystem.getAngle()));
      return !(angle > 90 || angle < -90);
    }
  }

  class HatchMode{
    boolean m_hatchMode;
    public HatchMode(boolean hatchMode){
      m_hatchMode = hatchMode;
    }

    public boolean condition(){
      return m_hatchMode;
    }
  }
  
  public RocketLoadingZoneCommand(FieldSide fieldSide, RocketSide rocketSide, ElevatorLevel level, boolean hatchMode) {
    if(fieldSide == FieldSide.Right){
      if(rocketSide == RocketSide.Close){
        addSequential(new ConditionCommand(new DriveBackwards(fieldSide, rocketSide), new CommandGroup(){
          {
            addParallel(new CommandGroup(){
              {
                addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
                addSequential(new ElevatorHomeCommand());
              }
            });
            addSequential(new CreatePathCommand(closeRightRocketToRightLoadingZonePath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
            // addSequential(new DrivePickUpBackUpHatchCommand(false, false, false));
            addSequential(hatchMode ? /*hatch*/ new DrivePickUpBackUpHatchCommand(false, false, false) : /*cargo*/ new DrivePickUpCargoCommand(false, true, false));//new DrivePickUpBackUpHatchCommand(false, false, false));
          }
        }, new CommandGroup(){
          {
            addParallel(new CommandGroup(){
              {
                addSequential(new ElevatorHomeCommand());
                addSequential(new ArmForwardCommand(false));
              }
            });
            addSequential(new CreatePathCommand(closeRightRocketToRightLoadingZonePath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, false));
            // addSequential(new DrivePickUpBackUpHatchCommand(false, false, true));
            addSequential(hatchMode ? /*hatch*/ new DrivePickUpBackUpHatchCommand(false, false, true) : /*cargo*/ new DrivePickUpCargoCommand(false, true, true));//new DrivePickUpBackUpHatchCommand(false, false, true));
          }
        }));
      }else if(rocketSide == RocketSide.Far){
        addSequential(new ConditionCommand(new DriveBackwards(fieldSide, rocketSide), new CommandGroup(){
          {
            addParallel(new CommandGroup(){
              {
                addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
                addSequential(new ElevatorHomeCommand());
              }
            });
            addSequential(new CreatePathCommand(farRocketToRightLoadingZoneV1P1Path, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false));
            addSequential(new CreatePathCommand(farRocketToRightLoadingZoneV1P2Path, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, false, false));
            // addSequential(new DrivePickUpBackUpHatchCommand(false, false, false));
            addSequential(hatchMode ? /*hatch*/ new DrivePickUpBackUpHatchCommand(false, false, true) : /*cargo*/ new DrivePickUpCargoCommand(false, false, true));//new DrivePickUpBackUpHatchCommand(false, false, false));
          }
        }, new CommandGroup(){
          {
            addParallel(new CommandGroup(){
              {
                addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
                addSequential(new ElevatorHomeCommand());
              }
            });
            addSequential(new CreatePathCommand(farRocketToRightLoadingZoneV1P1Path, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false));
            addSequential(new CreatePathCommand(farRocketToRightLoadingZoneV1P2Path, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, false, false));
            // addSequential(new DrivePickUpBackUpHatchCommand(false, false, true));
            addSequential(hatchMode ? /*hatch*/ new DrivePickUpBackUpHatchCommand(false, false, false) : /*cargo*/ new DrivePickUpCargoCommand(false, false, false));//new DrivePickUpBackUpHatchCommand(false, false, true));
          }
        }));
      }else if(rocketSide == RocketSide.Middle){
        addSequential(new ConditionCommand(new DriveBackwards(fieldSide, rocketSide), new CommandGroup(){
          {
            addParallel(new CommandGroup(){
              {
                addSequential(new CloseGrabberCommand());
                addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
                addSequential(new ElevatorHomeCommand());
              }
            });
            addSequential(new CreatePathCommand(middleRocketToRightLoadingZoneV2Path, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false));
            // addSequential(new DrivePickUpBackUpHatchCommand(false, false, false));
            addSequential(hatchMode ? /*hatch*/ new DrivePickUpBackUpHatchCommand(false, false, false) : /*cargo*/ new DrivePickUpCargoCommand(false, false, false));//new DrivePickUpBackUpHatchCommand(false, false, false));
          }
        }, new CommandGroup(){
          {
            addParallel(new CommandGroup(){
              {
                addSequential(new CloseGrabberCommand());
                addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
                addSequential(new ElevatorHomeCommand());
              }
            });
            addSequential(new CreatePathCommand(middleRocketToRightLoadingZoneV2Path, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false));
            // addSequential(new DrivePickUpBackUpHatchCommand(false, false, true));
            addSequential(hatchMode ? /*hatch*/ new DrivePickUpBackUpHatchCommand(false, false, true) : /*cargo*/ new DrivePickUpCargoCommand(false, false, true));//new DrivePickUpBackUpHatchCommand(false, false, true));
          }
        }));
      }
    }else{
      if(rocketSide == RocketSide.Close){
        addSequential(new ConditionCommand(new DriveBackwards(fieldSide, rocketSide), new CommandGroup(){
          {
            addParallel(new CommandGroup(){
              {
                addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
                addSequential(new ElevatorHomeCommand());
              }
            });
            addSequential(new CreatePathCommand(closeLeftRocketToLeftLoadingZonePath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
            // addSequential(new DrivePickUpBackUpHatchCommand(false, false, false));
            addSequential(hatchMode ? /*hatch*/ new DrivePickUpBackUpHatchCommand(false, false, false) : /*cargo*/ new DrivePickUpCargoCommand(false, true, false));//new DrivePickUpBackUpHatchCommand(false, false, false));
          }
        }, new CommandGroup(){
          {
            addParallel(new CommandGroup(){
              {
                addSequential(new ElevatorHomeCommand());
                addSequential(new ArmForwardCommand(false));
              }
            });
            addSequential(new CreatePathCommand(closeLeftRocketToLeftLoadingZonePath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false, false, false));
            // addSequential(new DrivePickUpBackUpHatchCommand(false, false, true));
            addSequential(hatchMode ? /*hatch*/ new DrivePickUpBackUpHatchCommand(false, false, true) : /*cargo*/ new DrivePickUpCargoCommand(false, true, true));//new DrivePickUpBackUpHatchCommand(false, false, true));
          }
        }));
      }
    }
  }
}
