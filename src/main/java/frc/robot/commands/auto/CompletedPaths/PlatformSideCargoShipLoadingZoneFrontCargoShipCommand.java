/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.CompletedPaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Navigator.CargoSide;
import frc.robot.Navigator.FieldSide;
import frc.robot.commands.auto.FrontCargoShipLeftLoadingZoneCommand;
import frc.robot.commands.auto.FrontCargoShipRightLoadingZoneCommand;
import frc.robot.commands.auto.LeftFrontLeftCargoShipCommand;
import frc.robot.commands.auto.LeftLoadingZoneSideCargoShipCommand;
import frc.robot.commands.auto.RightFrontRightCargoShipCommand;
import frc.robot.commands.auto.RightLoadingZoneSideCargoShipCommand;

public class PlatformSideCargoShipLoadingZoneFrontCargoShipCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PlatformSideCargoShipLoadingZoneFrontCargoShipCommand(FieldSide platformSide, CargoSide cargoSide) {
    boolean m_cameraCorrection = false;
    if(platformSide == FieldSide.Right){
      addSequential(new RightFrontRightCargoShipCommand());
      addSequential(new WaitCommand(1));
      addSequential(new FrontCargoShipRightLoadingZoneCommand(FieldSide.Right, m_cameraCorrection));
      addSequential(new WaitCommand(1));
      if(cargoSide == CargoSide.Close){
        addSequential(new RightLoadingZoneSideCargoShipCommand(CargoSide.Close));
      } else if(cargoSide == CargoSide.Middle){
        addSequential(new RightLoadingZoneSideCargoShipCommand(CargoSide.Middle));
      } else{
        addSequential(new RightLoadingZoneSideCargoShipCommand(CargoSide.Far));
      }
    } else{
      addSequential(new LeftFrontLeftCargoShipCommand());
      addSequential(new WaitCommand(1));
      addSequential(new FrontCargoShipLeftLoadingZoneCommand(FieldSide.Left, m_cameraCorrection));
      addSequential(new WaitCommand(1));
      if(cargoSide == CargoSide.Close){
        addSequential(new LeftLoadingZoneSideCargoShipCommand(CargoSide.Close));
      } else if(cargoSide == CargoSide.Middle){
        addSequential(new LeftLoadingZoneSideCargoShipCommand(CargoSide.Middle));
      } else{
        addSequential(new LeftLoadingZoneSideCargoShipCommand(CargoSide.Far));
      }
    }
  }
}
