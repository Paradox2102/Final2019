package frc.robot.commands.auto.autoTasks;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.lib.Condition;
import frc.lib.ConditionCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.LED.SetClimbLEDCommand;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.climber.ActivatePistonsCommand;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.grabber.CloseGrabberCommand;
import frc.robot.commands.grabber.OpenGrabberCommand;
import frc.robot.commands.intake.IntakeClimbCommand;
import frc.robot.commands.intake.MoveIntakePosCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbCommand extends CommandGroup {

  class MoveElevator implements Condition{
    public boolean condition(){
      if(RobotMap.k_finalRobot){
        return Robot.m_intakeSubsystem.getAngleFinal() > 90;
      }
      return Robot.m_intakeSubsystem.getAngle() > 90;
    }
  }
  public ClimbCommand() {
    addSequential(new SetClimbLEDCommand(true));
    addSequential(new SetClimbLEDCommand(true));
    addSequential(new CloseGrabberCommand());
    addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    addSequential(new OpenGrabberCommand());
    addSequential(new ConditionCommand(new MoveElevator(), new MoveElevatorPosCommand(ElevatorLevel.ClimbHeight), new DoNothingCommand()));
    if(RobotMap.k_finalRobot){
      addSequential(new MoveIntakePosCommand(IntakeSubsystem.ticksToDegFinal(RobotMap.k_intakeClimbPos), RobotMap.k_finalRobot));
    }else{
      addSequential(new MoveIntakePosCommand(IntakeSubsystem.ticksToDeg(RobotMap.k_intakeClimbPos), RobotMap.k_finalRobot));
    }
    addSequential(new ElevatorHomeCommand());
    addSequential(new CloseGrabberCommand());
    addParallel(new IntakeClimbCommand(0.75));
    addSequential(new CommandGroup() {
      {
        addSequential(new WaitCommand(1));
        addSequential(new ActivatePistonsCommand());
      }
    });
  }
}
