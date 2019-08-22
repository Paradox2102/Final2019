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
import frc.lib.PiCamera.TargetSelection;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Navigator.CameraDirection;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.drive.StallDriveCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.elevator.MoveElevatorPosCommand.RobotMode;
import frc.robot.commands.grabber.CloseGrabberCommand;
import frc.robot.commands.grabberWheels.OuttakeBurpCommand;
import frc.robotCore.Logger;

public class DriveDropOffBackUpHatchCommand extends CommandGroup {
  boolean m_driverInput;
  class DroppedOffHatch implements Condition{
    public boolean condition(){
      return !Robot.m_armSubsystem.hasHatch();
    }
  }

  class HatchMode implements Condition{
    public RobotMode m_mode;

    public HatchMode(){
      m_mode = RobotMode.Hatch;
    }

    public HatchMode(RobotMode mode){
      m_mode = mode;
    }
    public boolean condition(){
      if(m_driverInput){
        Logger.Log("Hatch Mode", 2, String.format("Condition: %b", Robot.m_oi.hatchMode()));
        return Robot.m_oi.hatchMode();
      }else{
        Logger.Log("Hatch Mode", 2, String.format("Condition: %b", m_mode == RobotMode.Hatch));
        return m_mode == RobotMode.Hatch;
      }
    }
  }

  class ElevatorMove implements Condition{
    ElevatorLevel m_level;
    public ElevatorMove(ElevatorLevel level){
      m_level = level;
    }

    public boolean condition(){
      return m_level != ElevatorLevel.Level1;
    }
  }
  public DriveDropOffBackUpHatchCommand(boolean backUp, boolean backward, boolean driverInput, TargetSelection target) {
    m_driverInput = driverInput;
    addParallel(new ElevatorHomeCommand());
    addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    if(backward){
      addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false, target, false, 0));
      addSequential(new ConditionCommand(new HatchMode(), new DoNothingCommand(), new MoveElevatorPosCommand(ElevatorLevel.CargoShip)));
      addSequential(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot));
      addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
      addSequential(new ConditionCommand(new HatchMode(), new CloseGrabberCommand(), new OuttakeBurpCommand(1)));
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
      }
    }else{
      addSequential(new ConditionCommand(new HatchMode(), new CommandGroup(){
        {
          addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front, true, target, false, 0));
          addSequential(new ArmForwardCommand(false));
        }
      }, new CommandGroup(){
        {
          addParallel(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front, true, target, false, 0));
          addSequential(new MoveElevatorPosCommand(ElevatorLevel.CargoShip));
          addSequential(new MoveArmPosCommand(ArmPos.CargoForward, RobotMap.k_finalRobot));
        }
      }));
      addSequential(new PowerDriveByTimeCommand(0.15, 0.75));
      addSequential(new ConditionCommand(new HatchMode(), new CloseGrabberCommand(), new OuttakeBurpCommand(1)));
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(-0.15, 0.5));
      }
    }
    
  }

  public DriveDropOffBackUpHatchCommand(boolean backUp, boolean backward, boolean driverInput, TargetSelection target, boolean armUp, boolean earlyEnd, double endingPoint) {
    m_driverInput = driverInput;
    addParallel(new ElevatorHomeCommand());
    if(armUp){
      addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    }
    if(backward){
      addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false, target, earlyEnd, endingPoint));
      addSequential(new ConditionCommand(new HatchMode(), new DoNothingCommand(), new MoveElevatorPosCommand(ElevatorLevel.CargoShip)));
      addSequential(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot));
      addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
      addSequential(new ConditionCommand(new HatchMode(), new CloseGrabberCommand(), new OuttakeBurpCommand(1)));
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
      }
    }else{
      addSequential(new ConditionCommand(new HatchMode(), new CommandGroup(){
        {
          addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front, true, target, earlyEnd, endingPoint));
          addSequential(new ArmForwardCommand(false));
        }
      }, new CommandGroup(){
        {
          addParallel(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front, true, target, earlyEnd, endingPoint));
          addSequential(new MoveElevatorPosCommand(ElevatorLevel.CargoShip));
          addSequential(new MoveArmPosCommand(ArmPos.CargoForward, RobotMap.k_finalRobot));
        }
      }));
      addSequential(new PowerDriveByTimeCommand(0.15, 0.75));
      addSequential(new ConditionCommand(new HatchMode(), new CloseGrabberCommand(), new OuttakeBurpCommand(1)));
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(-0.15, 0.5));
      }
    }
    
  }

  public DriveDropOffBackUpHatchCommand(boolean backUp, boolean backward, boolean driverInput, ElevatorLevel level) {
    // m_driverInput = driverInput;
    // addParallel(new ElevatorHomeCommand());
    // addSequential(new MoveArmPosCommand(90, RobotMap.k_finalRobot));
    // if(backward){
    //   addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
    //   addSequential(new MoveElevatorPosCommand(level));
    //   addSequential(new MoveArmPosCommand(180, RobotMap.k_finalRobot, true));
    //   addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
    //   addSequential(new ConditionCommand(new HatchMode(), new CloseGrabberCommand(), new CommandGroup(){
    //     {
    //       addSequential(new OuttakeBurpCommand(1));
    //       addSequential(new CloseGrabberCommand());
    //     }
    //   }));
    //   if(backUp){
    //     addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
    //   }
    // }else{
    //   addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front, true));
    //   addSequential(new MoveElevatorPosCommand(level));
    //   addSequential(new ArmForwardCommand(true));
    //   addSequential(new PowerDriveByTimeCommand(0.15, 0.75));
    //   addSequential(new ConditionCommand(new HatchMode(), new CloseGrabberCommand(), new CommandGroup(){
    //     {
    //       addSequential(new OuttakeBurpCommand(1));
    //       addSequential(new CloseGrabberCommand());
    //     }
    //   }));
      
    //   if(backUp){
    //     addSequential(new PowerDriveByTimeCommand(-0.15, 0.5));
    //   }
    // }
    m_driverInput = driverInput;
    RobotMode robotMode = RobotMode.driverInput;
    addParallel(new ElevatorHomeCommand());
    addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    if(backward){
      addSequential(new ConditionCommand(new ElevatorMove(level), new CommandGroup(){
        {
          addParallel(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
          addSequential(new MoveElevatorPosCommand(level, robotMode));
          addSequential(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot, true));
          // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
          addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
          addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
            {
              addSequential(new OuttakeBurpCommand(1));
              addSequential(new CloseGrabberCommand());
            }
          }));
          if(backUp){
            addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
          }
        }
      }, new CommandGroup(){
        {
          addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
          addSequential(new MoveElevatorPosCommand(level, robotMode));
          addSequential(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot, true));
          // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
          addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
          addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
            {
              addSequential(new OuttakeBurpCommand(1));
              addSequential(new CloseGrabberCommand());
            }
          }));
          if(backUp){
            addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
          }
        }
      }));
      // addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
      // addSequential(new MoveElevatorPosCommand(level, robotMode));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
      // addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
      //   {
      //     addSequential(new OuttakeBurpCommand(1));
      //     addSequential(new CloseGrabberCommand());
      //   }
      // }));
      // if(backUp){
      //   addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
      // }
    }else{
      addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front, true));
      addSequential(new MoveElevatorPosCommand(level, robotMode));
      addSequential(new ArmForwardCommand(true));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new ArmForwardCommand(false), new ArmForwardCommand(false, 10)));
      addSequential(new PowerDriveByTimeCommand(0.15, 0.75));
      addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
        {
          addSequential(new OuttakeBurpCommand(1));
          addSequential(new CloseGrabberCommand());
        }
      }));
      
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(-0.15, 0.5));
      }
    }
  }

  public DriveDropOffBackUpHatchCommand(boolean backUp, boolean backward, boolean driverInput, ElevatorLevel level, RobotMode robotMode) {
    m_driverInput = driverInput;
    addParallel(new ElevatorHomeCommand());
    addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    if(backward){
      addSequential(new ConditionCommand(new ElevatorMove(level), new CommandGroup(){
        {
          addParallel(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
          addParallel(new MoveElevatorPosCommand(level, robotMode));
          addSequential(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot, true));
          // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
          addSequential(new PowerDriveByTimeCommand(-0.15, 1));
          addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
            {
              addSequential(new OuttakeBurpCommand(1));
              addSequential(new CloseGrabberCommand());
            }
          }));
          if(backUp){
            addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
          }
        }
      }, new CommandGroup(){
        {
          addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
          addSequential(new MoveElevatorPosCommand(level, robotMode));
          addSequential(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot, true));
          // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
          addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
          addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
            {
              addSequential(new OuttakeBurpCommand(1));
              addSequential(new CloseGrabberCommand());
            }
          }));
          if(backUp){
            addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
          }
        }
      }));
      // addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
      // addSequential(new MoveElevatorPosCommand(level, robotMode));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
      // addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
      //   {
      //     addSequential(new OuttakeBurpCommand(1));
      //     addSequential(new CloseGrabberCommand());
      //   }
      // }));
      // if(backUp){
      //   addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
      // }
    }else{
      addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front, true));
      addSequential(new MoveElevatorPosCommand(level, robotMode));
      addSequential(new ArmForwardCommand(true));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new ArmForwardCommand(false), new ArmForwardCommand(false, 10)));
      addSequential(new PowerDriveByTimeCommand(0.15, 0.75));
      addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
        {
          addSequential(new OuttakeBurpCommand(1));
          addSequential(new CloseGrabberCommand());
        }
      }));
      
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(-0.15, 0.5));
      }
    }
    
  }

  public DriveDropOffBackUpHatchCommand(boolean backUp, boolean backward, boolean driverInput, ElevatorLevel level, RobotMode robotMode, double time) {
    m_driverInput = driverInput;
    addParallel(new ElevatorHomeCommand());
    addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    if(backward){
      addSequential(new ConditionCommand(new ElevatorMove(level), new CommandGroup(){
        {
          addParallel(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
          addParallel(new MoveElevatorPosCommand(level, robotMode));
          addSequential(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot, true));
          // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
          addSequential(new PowerDriveByTimeCommand(-0.15, 1));
          addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
            {
              addSequential(new OuttakeBurpCommand(1));
              addSequential(new CloseGrabberCommand());
            }
          }));
          if(backUp){
            addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
          }
        }
      }, new CommandGroup(){
        {
          addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
          addSequential(new MoveElevatorPosCommand(level, robotMode));
          addSequential(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot, true));
          // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
          addSequential(new PowerDriveByTimeCommand(-0.15, time));
          addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
            {
              addSequential(new OuttakeBurpCommand(1));
              addSequential(new CloseGrabberCommand());
            }
          }));
          if(backUp){
            addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
          }
        }
      }));
      // addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
      // addSequential(new MoveElevatorPosCommand(level, robotMode));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new MoveArmPosCommand(180, RobotMap.k_finalRobot, false), new MoveArmPosCommand(175, RobotMap.k_finalRobot, false)));
      // addSequential(new PowerDriveByTimeCommand(-0.15, 0.75));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
      //   {
      //     addSequential(new OuttakeBurpCommand(1));
      //     addSequential(new CloseGrabberCommand());
      //   }
      // }));
      // if(backUp){
      //   addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
      // }
    }else{
      addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front, true));
      addSequential(new MoveElevatorPosCommand(level, robotMode));
      addSequential(new ArmForwardCommand(true));
      // addSequential(new ConditionCommand(new HatchMode(robotMode), new ArmForwardCommand(false), new ArmForwardCommand(false, 10)));
      addSequential(new PowerDriveByTimeCommand(0.15, 0.75));
      addSequential(new ConditionCommand(new HatchMode(robotMode), new CloseGrabberCommand(), new CommandGroup(){
        {
          addSequential(new OuttakeBurpCommand(1));
          addSequential(new CloseGrabberCommand());
        }
      }));
      
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(-0.15, 0.5));
      }
    }
    
  }
}
