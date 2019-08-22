/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.CompletedPaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.PiCamera.TargetSelection;
import frc.robot.RobotMap;
import frc.robot.Navigator.FieldSide;
import frc.robot.Navigator.RocketSide;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.DriveDropOffBackUpHatchCommand;
import frc.robot.commands.auto.DrivePickUpBackUpHatchCommand;
import frc.robot.commands.auto.FrontCargoShipRightLoadingZoneCommand;
import frc.robot.commands.auto.LoadingZoneRocketCommand;
import frc.robot.commands.auto.RightFrontRightCargoShipCommand;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;

public class PlatformFrontCargoShipLoadingZoneRocketCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PlatformFrontCargoShipLoadingZoneRocketCommand(FieldSide fieldSide, RocketSide rocketSide, ElevatorLevel rocketLevel) {
    addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    if(fieldSide == FieldSide.Right){
      addSequential(new RightFrontRightCargoShipCommand());
      addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, TargetSelection.right));
      addParallel(new ArmForwardCommand(false));
      addSequential(new FrontCargoShipRightLoadingZoneCommand(FieldSide.Right, false));
      addSequential(new DrivePickUpBackUpHatchCommand(false, true, true));
      addSequential(new LoadingZoneRocketCommand(FieldSide.Right, rocketSide, rocketLevel));
      // if(cargoSide == CargoSide.Close){
      //   addSequential(new RightLoadingZoneSideCargoShipCommand(CargoSide.Close));
      // } else if(cargoSide == CargoSide.Middle){
      //   addSequential(new RightLoadingZoneSideCargoShipCommand(CargoSide.Middle));
      // } else{
      //   addSequential(new RightLoadingZoneSideCargoShipCommand(CargoSide.Far));
      // }
      // addSequential(new DriveDropOffBackUpHatchCommand(true, true, false));
    } else{
      // addSequential(new LeftFrontLeftCargoShipCommand());
      // addSequential(new DriveDropOffBackUpHatchCommand(false, true, false));
      // addSequential(new FrontCargoShipLeftLoadingZoneCommand(FieldSide.Left, m_cameraCorrection));
      // addSequential(new DrivePickUpBackUpHatchCommand(false, true, true));
      // if(cargoSide == CargoSide.Close){
      //   addSequential(new LeftLoadingZoneSideCargoShipCommand(CargoSide.Close));
      // } else if(cargoSide == CargoSide.Middle){
      //   addSequential(new LeftLoadingZoneSideCargoShipCommand(CargoSide.Middle));
      // } else{
      //   addSequential(new LeftLoadingZoneSideCargoShipCommand(CargoSide.Far));
      // }
      // addSequential(new DriveDropOffBackUpHatchCommand(false, true, false));
    }
  }
}
