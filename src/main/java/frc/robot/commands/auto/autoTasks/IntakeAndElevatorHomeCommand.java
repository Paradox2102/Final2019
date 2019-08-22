/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.arm.WaitForArmCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.intake.IntakeHomeCommand;

public class IntakeAndElevatorHomeCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public IntakeAndElevatorHomeCommand(double armPos, double intakePower) {
    addSequential(new WaitForArmCommand(armPos, 2));
    addSequential(new IntakeHomeCommand(intakePower));
    addSequential(new ElevatorHomeCommand());
  }
}
