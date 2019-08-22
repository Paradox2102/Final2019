/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.CompletedPaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.lib.PiCamera.TargetSelection;
import frc.robot.RobotMap;
import frc.robot.Navigator.FieldSide;
import frc.robot.Navigator.RocketSide;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.CenterFrontCargoShipCommand;
import frc.robot.commands.auto.DriveDropOffBackUpHatchCommand;
import frc.robot.commands.auto.DrivePickUpBackUpHatchCommand;
import frc.robot.commands.auto.FrontCargoShipLeftLoadingZoneCommand;
import frc.robot.commands.auto.FrontCargoShipRightLoadingZoneCommand;
import frc.robot.commands.auto.LoadingZoneFrontCargoShipCommand;
import frc.robot.commands.auto.LoadingZoneRocketCommand;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;

public class CenterFrontCargoShipLoadingZoneFrontRocketCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CenterFrontCargoShipLoadingZoneFrontRocketCommand(FieldSide fieldSide, ElevatorLevel level) {
    if(fieldSide == FieldSide.Right){
      addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
      // addSequential(new CreatePathCommand(centerPlatformFrontCargoShip, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
      addSequential(new CenterFrontCargoShipCommand(FieldSide.Right, false));
      addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, TargetSelection.right));
      addParallel(new CommandGroup(){
        {
          addSequential(new WaitCommand(1));
          addSequential(new ArmForwardCommand(false));
        }
      });
      addSequential(new FrontCargoShipRightLoadingZoneCommand(FieldSide.Right, false));
      addSequential(new DrivePickUpBackUpHatchCommand(false, true, true));
      addSequential(new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Close, level));
      // addSequential(new CenterFrontCargoShipCommand(FieldSide.Left, m_cameraCorrection));  
      // addSequential(new MoveArmPosCommand(0, false));
      // addSequential(new DriveDropOffBackUpHatchCommand());
      // addSequential(new FrontCargoShipRightLoadingZoneCommand(FieldSide.Right, m_cameraCorrection));
      // addSequential(new DrivePickUpBackUpHatchCommand(true, true, true));
      // addSequential(new WaitCommand(1));
      // addSequential(new LoadingZoneFrontCargoShipCommand(FieldSide.Right, m_cameraCorrection));
    } else{
      addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
      // addSequential(new CreatePathCommand(centerPlatformFrontCargoShip, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
      addSequential(new CenterFrontCargoShipCommand(FieldSide.Left, false));
      addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, TargetSelection.left));
      addParallel(new CommandGroup(){
        {
          addSequential(new WaitCommand(1));
          addSequential(new ArmForwardCommand(false));
        }
      });
      addSequential(new FrontCargoShipLeftLoadingZoneCommand(FieldSide.Left, false));
      addSequential(new DrivePickUpBackUpHatchCommand(false, true, true));
      addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
      addSequential(new LoadingZoneRocketCommand(FieldSide.Left, RocketSide.Close, level));
    }
  }
}
