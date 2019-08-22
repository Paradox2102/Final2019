package frc.robot.commands.auto.CompletedPaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.lib.PiCamera.TargetSelection;
import frc.robot.RobotMap;
import frc.robot.Navigator.FieldSide;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.CenterFrontCargoShipCommand;
import frc.robot.commands.auto.DriveDropOffBackUpHatchCommand;
import frc.robot.commands.auto.DriveDropOffBackUpHatchFastCommand;
import frc.robot.commands.auto.DrivePickUpBackUpHatchCommand;
import frc.robot.commands.auto.FrontCargoShipLeftLoadingZoneCommand;
import frc.robot.commands.auto.FrontCargoShipRightLoadingZoneCommand;
import frc.robot.commands.auto.LoadingZoneFrontCargoShipCommand;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;

public class CenterFrontCargoShipLoadingZoneFrontCargoShipCommand extends CommandGroup {
  boolean m_cameraCorrection = false;
  public CenterFrontCargoShipLoadingZoneFrontCargoShipCommand(FieldSide fieldSide) {
    if(fieldSide == FieldSide.Right){
      addParallel(new MoveArmPosCommand(ArmPos.HatchCameraAvoid, RobotMap.k_finalRobot));
      // addSequential(new CreatePathCommand(centerPlatformFrontCargoShip, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, false));
      addSequential(new CenterFrontCargoShipCommand(FieldSide.Right, false));
      addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, TargetSelection.right, false, false, 0));
      addParallel(new CommandGroup(){
        {
          addSequential(new WaitCommand(1));
          addSequential(new ArmForwardCommand(false));
        }
      });
      addSequential(new FrontCargoShipRightLoadingZoneCommand(FieldSide.Right, false));
      addSequential(new DrivePickUpBackUpHatchCommand(false, true, true));
      addParallel(new MoveArmPosCommand(ArmPos.HatchCameraAvoid, RobotMap.k_finalRobot));
      addParallel(new ElevatorHomeCommand());
      addSequential(new LoadingZoneFrontCargoShipCommand(FieldSide.Right, FieldSide.Left, false));
      addSequential(new DriveDropOffBackUpHatchFastCommand(false, true, false, 1000, false, TargetSelection.left, false, 0));
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
          addSequential(new WaitCommand(0.5));
          addSequential(new ArmForwardCommand(false));
        }
      });
      addSequential(new FrontCargoShipLeftLoadingZoneCommand(FieldSide.Left, false));//moved ending point over 0.5ft
      addParallel(new ArmForwardCommand(false));
      addSequential(new DrivePickUpBackUpHatchCommand(false, true, true));
      addParallel(new MoveArmPosCommand(ArmPos.HatchCameraAvoid, RobotMap.k_finalRobot));
      addParallel(new ElevatorHomeCommand());
      addSequential(new LoadingZoneFrontCargoShipCommand(FieldSide.Left, FieldSide.Right, false));//moved starting point 0.5
      addSequential(new DriveDropOffBackUpHatchFastCommand(false, true, false, 1000, false, TargetSelection.right, false, 0));
    }
  }
}