/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.lib.Condition;
import frc.lib.ConditionCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.WaitForArmCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.grabber.OpenGrabberCommand;
import frc.robot.commands.intake.MoveIntakePosCommand;
import frc.robot.commands.intake.WaitForIntakeCommand;

public class DeployIntakeCommand extends CommandGroup {
  private class IntakeClear implements Condition{
    double m_clearing;
    public IntakeClear(double clearing){
      m_clearing = clearing;
    }
    public boolean condition(){
      // if(RobotMap.k_finalRobot){
      //   return Robot.m_intakeSubsystem.getAngleFinal() < m_clearing;
      // }else{
        return Robot.m_intakeSubsystem.getAngle() < m_clearing;
      // }
    }
  }
  public DeployIntakeCommand(double intakePos) {
    addParallel(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot, false));
    addSequential(new MoveElevatorPosCommand(ElevatorLevel.CargoAvoidanceLevel));
    addParallel(new CommandGroup(){
      {
        addSequential(new ConditionCommand(new IntakeClear(80), new DoNothingCommand(), new WaitForIntakeCommand(80, 2)));
        addSequential(new OpenGrabberCommand());
        addParallel(new MoveArmPosCommand(ArmPos.Intake, RobotMap.k_finalRobot, false));
        addSequential(new WaitForArmCommand(RobotMap.k_armIntakeAngle, 10));
        addSequential(new MoveElevatorPosCommand(ElevatorLevel.IntakeLevel));
      }
    });
    addSequential(new MoveIntakePosCommand(intakePos, RobotMap.k_finalRobot));
  }
}
