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
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.AngledFrontLeftLoadingZoneCommand;
import frc.robot.commands.auto.AngledLeftLoadingZoneLeftSideCargoShipCommand;
import frc.robot.commands.auto.DriveDropOffBackUpHatchCommand;
import frc.robot.commands.auto.DriveDropOffBackUpHatchFastCommand;
import frc.robot.commands.auto.DrivePickUpBackUpHatchCommand;
import frc.robot.commands.auto.LeftFrontLeftCargoShipCommand;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;

public class LeftFrontLeftCargoShipLoadingZoneSideCargoShipCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftFrontLeftCargoShipLoadingZoneSideCargoShipCommand() {
    addParallel(new MoveArmPosCommand(ArmPos.HatchCameraAvoid, RobotMap.k_finalRobot));
    addSequential(new LeftFrontLeftCargoShipCommand());
    addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, TargetSelection.left, false, false, 0));
    addParallel(new CommandGroup(){
      {
        addSequential(new WaitCommand(0.25));
        addSequential(new ArmForwardCommand(false));
      }
    });
    addSequential(new AngledFrontLeftLoadingZoneCommand());
    addSequential(new ArmForwardCommand(false));
    addSequential(new DrivePickUpBackUpHatchCommand(false, true, true));
    addParallel(new MoveArmPosCommand(ArmPos.HatchCameraAvoid, RobotMap.k_finalRobot));
    addSequential(new AngledLeftLoadingZoneLeftSideCargoShipCommand());
    addSequential(new DriveDropOffBackUpHatchFastCommand(false, true, false, 1000, false, TargetSelection.right, false, 0));
  }
}
