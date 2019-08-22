package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.MoveElevatorPosCommand.StallCase;

public class AvoidIntakeCommand extends Command {
  double m_power;
  boolean m_end;
  StallCase m_stallCase;
  long m_startTime;
  long k_stallTime = 250;
  int k_error = 100;
  
  public AvoidIntakeCommand(double power) {
    requires(Robot.m_elevatorSubsystem);

    m_power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    m_stallCase = StallCase.checkEnd;
    m_startTime = System.currentTimeMillis();
    m_end = false;
    if(Robot.m_intakeSubsystem.getLimit() || Robot.m_intakeSubsystem.getAngle() > 90){
      Robot.m_elevatorSubsystem.setPos(RobotMap.k_elevatorIntakeAvoidHeight);
    }else{
      m_end = true;
    }
    // if(Robot.m_elevatorSubsystem.getPos() > RobotMap.k_elevatorIntakeAvoidHeight){
    //   m_end = true;
    // }else{
    //   Robot.m_elevatorSubsystem.setPos(RobotMap.k_elevatorIntakeAvoidHeight);
    // }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_end){
      return true;
    }
    switch (m_stallCase){
      case checkEnd:
        boolean finished = false;
        finished = Robot.m_elevatorSubsystem.getPosRelativeHome() > RobotMap.k_elevatorIntakeAvoidHeight - k_error;
        
        if(finished){
          m_startTime = System.currentTimeMillis();
          m_stallCase = StallCase.stalling;
        }
        break;
      case stalling:
        if(m_startTime + k_stallTime < System.currentTimeMillis()){
          Robot.m_elevatorSubsystem.brake();
          m_stallCase = StallCase.braked;
          m_startTime = System.currentTimeMillis();
        }
        break;
      case braked:
        if(m_startTime + k_stallTime < System.currentTimeMillis()){
          return true;
        }
        break;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_elevatorSubsystem.stop();
  }
}
