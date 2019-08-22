/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.lib.Condition;
import frc.lib.ConditionCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Navigator.CameraDirection;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.grabber.CloseGrabberCommand;
import frc.robot.commands.grabber.OpenGrabberCommand;

public class DrivePickUpBackUpHatchCommand extends CommandGroup {
  class HasHatch implements Condition {
    boolean m_auto;
    public HasHatch(boolean auto){
      m_auto = auto;
    }
    public boolean condition(){
      return Robot.m_armSubsystem.hasHatch() || m_auto;
    }
  }
  public DrivePickUpBackUpHatchCommand(boolean backUp, boolean auto, boolean front) {
    addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    addParallel(new ElevatorHomeCommand());
    addSequential(new CloseGrabberCommand());
    if(front){
      addParallel(new ArmForwardCommand(false));
      addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front));
      addSequential(new PowerDrivePickUpHatchCommand(0.1));
      addSequential(new ConditionCommand(new HasHatch(true), new CommandGroup(){
        {
          // addSequential(new StallDriveCommand(0.1));
          addParallel(new OpenGrabberCommand());
          addSequential(new MoveElevatorPosCommand(ElevatorLevel.FeederPlaceHeight));
          // // addSequential(new WaitCommand(0.5));
          // addParallel(new MoveArmPosCommand(90, RobotMap.k_finalRobot, false));
          if(backUp){
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new PowerDriveByTimeCommand(-0.15, 1));
          }
        }
      }, new DoNothingCommand()));
    }else{
      addParallel(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot));
      addSequential(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Back, false));
      addSequential(new PowerDrivePickUpHatchCommand(-0.1));
      addSequential(new ConditionCommand(new HasHatch(true), new CommandGroup(){
        {
          // addSequential(new StallDriveCommand(0.1));
          addParallel(new OpenGrabberCommand());
          addSequential(new MoveElevatorPosCommand(ElevatorLevel.FeederPlaceHeight));
          // // addSequential(new WaitCommand(0.5));
          // addParallel(new MoveArmPosCommand(90, RobotMap.k_finalRobot, false));
          if(backUp){
            addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
            addSequential(new PowerDriveByTimeCommand(0.15, 1));
          }
        }
      }, new DoNothingCommand()));
    }
  }
}
