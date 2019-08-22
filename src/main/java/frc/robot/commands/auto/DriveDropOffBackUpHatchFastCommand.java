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

public class DriveDropOffBackUpHatchFastCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
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
  public DriveDropOffBackUpHatchFastCommand(boolean backUp, boolean backward, boolean driverInput, double driveSpeed, boolean armUp, TargetSelection target, boolean earlyEnd, double endingPoint) {
    m_driverInput = driverInput;
    addParallel(new ElevatorHomeCommand());
    if(armUp){
      addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    }
    if(backward){
      addSequential(new CameraAngleDriveCommand(driveSpeed, 2.0/12.0, CameraDirection.Back, false, target, earlyEnd, endingPoint));
      addSequential(new ConditionCommand(new HatchMode(), new DoNothingCommand(), new MoveElevatorPosCommand(ElevatorLevel.CargoShip)));
      addParallel(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot));
      addSequential(new PowerDriveByTimeCommand(-0.1, 1));
      addSequential(new ConditionCommand(new HatchMode(), new DoNothingCommand(), new OuttakeBurpCommand(1)));
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
      }
    }else{
      addSequential(new ConditionCommand(new HatchMode(), new CommandGroup(){
        {
          addSequential(new CameraAngleDriveCommand(driveSpeed, 2.0/12.0, CameraDirection.Front, true, target, false, 0));
          addSequential(new ArmForwardCommand(false));
        }
      }, new CommandGroup(){
        {
          addParallel(new CameraAngleDriveCommand(driveSpeed, 2.0/12.0, CameraDirection.Front, true, target, false, 0));
          addSequential(new MoveElevatorPosCommand(ElevatorLevel.CargoShip));
          addSequential(new MoveArmPosCommand(ArmPos.CargoForward, RobotMap.k_finalRobot));
        }
      }));
      addSequential(new PowerDriveByTimeCommand(0.1, 1));
      addSequential(new ConditionCommand(new HatchMode(), new DoNothingCommand(), new OuttakeBurpCommand(1)));
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(-0.15, 0.5));
      }
    }
    
  }

  public DriveDropOffBackUpHatchFastCommand(boolean backUp, boolean backward, boolean driverInput, double driveSpeed, boolean armUp, TargetSelection target, boolean release) {
    m_driverInput = driverInput;
    addParallel(new ElevatorHomeCommand());
    if(armUp){
      addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    }
    if(backward){
      addSequential(new CameraAngleDriveCommand(driveSpeed, 2.0/12.0, CameraDirection.Back, false, target, false, 0));
      addSequential(new ConditionCommand(new HatchMode(), new DoNothingCommand(), new MoveElevatorPosCommand(ElevatorLevel.CargoShip)));
      addParallel(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot));
      addSequential(new PowerDriveByTimeCommand(-0.1, 1));
      addSequential(new ConditionCommand(new HatchMode(), release ? new CloseGrabberCommand() : new DoNothingCommand(), new OuttakeBurpCommand(1)));
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(0.15, 0.5));
      }
    }else{
      addSequential(new ConditionCommand(new HatchMode(), new CommandGroup(){
        {
          addSequential(new CameraAngleDriveCommand(driveSpeed, 2.0/12.0, CameraDirection.Front, true, target, false, 0));
          addSequential(new ArmForwardCommand(false));
        }
      }, new CommandGroup(){
        {
          addParallel(new CameraAngleDriveCommand(driveSpeed, 2.0/12.0, CameraDirection.Front, true, target, false, 0));
          addSequential(new MoveElevatorPosCommand(ElevatorLevel.CargoShip));
          addSequential(new MoveArmPosCommand(ArmPos.CargoForward, RobotMap.k_finalRobot));
        }
      }));
      addSequential(new PowerDriveByTimeCommand(0.1, 1));
      addSequential(new ConditionCommand(new HatchMode(), release ? new CloseGrabberCommand() : new DoNothingCommand(), new OuttakeBurpCommand(1)));
      if(backUp){
        addSequential(new PowerDriveByTimeCommand(-0.15, 0.5));
      }
    }
    
  }
}
