package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robotCore.Logger;

public class MoveElevatorPosCommand extends Command {
  ElevatorLevel m_level;
  double m_pos = 0;
  boolean m_up;
  StallCase m_stallCase;
  long m_startTime;
  long k_stallTime = 0;//250;
  int k_error = 50;
  RobotMode m_robotMode;

  public enum ElevatorLevel{
    Level1, Level2, Level3, CargoShip, CargoAvoidanceLevel, IntakeLevel, ClimbHeight, ArmAvoid, FeederPlaceHeight, FeederGrabHeight
  };

  public enum RobotMode{
    driverInput, Cargo, Hatch
  };

  public enum StallCase {
    checkEnd, stalling, braked
  };

  public MoveElevatorPosCommand(ElevatorLevel level) {
    requires(Robot.m_elevatorSubsystem);

    m_robotMode = RobotMode.driverInput;
    m_level = level;
  }

  public MoveElevatorPosCommand(ElevatorLevel level, RobotMode mode) {
    requires(Robot.m_elevatorSubsystem);

    m_robotMode = mode;
    m_level = level;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.Log("MoveElevatorPosCommand", 3, "Init");

    setTimeout(3);

    m_stallCase = StallCase.checkEnd;
    m_startTime = System.currentTimeMillis();
   
    if(m_robotMode == RobotMode.driverInput){
      if(Robot.m_oi.hatchMode()){
        if(m_level == ElevatorLevel.Level1){
          m_pos = RobotMap.k_rocketHatchLevel1;
        }else if(m_level == ElevatorLevel.Level2){
          m_pos = RobotMap.k_rocketHatchLevel2;
        }else if(m_level == ElevatorLevel.Level3){
          m_pos = RobotMap.k_rocketHatchLevel3;
        }
      }else{
        if(m_level == ElevatorLevel.Level1){
          m_pos = RobotMap.k_rocketCargoLevel1;
        }else if(m_level == ElevatorLevel.Level2){
          m_pos = RobotMap.k_rocketCargoLevel2;
        }else if(m_level == ElevatorLevel.Level3){
          m_pos = RobotMap.k_rocketCargoLevel3;
        }
      }
  
      if(m_level == ElevatorLevel.CargoShip){
        m_pos = RobotMap.k_elevatorCargoShip;
      }
  
      if(m_level == ElevatorLevel.CargoAvoidanceLevel){
        m_pos = RobotMap.k_elevatorIntakeAvoidHeight;
      }
  
      if(m_level == ElevatorLevel.IntakeLevel){
        m_pos = RobotMap.k_elevatorIntakeHeight;
      }
  
      if(m_level == ElevatorLevel.ClimbHeight){
        m_pos = RobotMap.k_climbHeight;
      }
  
      if(m_level == ElevatorLevel.ArmAvoid){
        m_pos = RobotMap.k_elevatorArmAvoidHeight;
      }
  
      if(m_level == ElevatorLevel.FeederPlaceHeight){
        m_pos = RobotMap.k_elevatorFeederPlacement;
      }

      if(m_level == ElevatorLevel.FeederGrabHeight){
        m_pos = RobotMap.k_elevatorGrabHeight;
      }
    }else if(m_robotMode == RobotMode.Hatch){
      if(m_level == ElevatorLevel.Level1){
        m_pos = RobotMap.k_rocketHatchLevel1;
      }else if(m_level == ElevatorLevel.Level2){
        m_pos = RobotMap.k_rocketHatchLevel2;
      }else if(m_level == ElevatorLevel.Level3){
        m_pos = RobotMap.k_rocketHatchLevel3;
      }
    }else if(m_robotMode == RobotMode.Cargo){
      if(m_level == ElevatorLevel.Level1){
        m_pos = RobotMap.k_rocketCargoLevel1;
      }else if(m_level == ElevatorLevel.Level2){
        m_pos = RobotMap.k_rocketCargoLevel2;
      }else if(m_level == ElevatorLevel.Level3){
        m_pos = RobotMap.k_rocketCargoLevel3;
      }
    }

    m_up = m_pos > Robot.m_elevatorSubsystem.getPosRelativeHome();
    
    if(m_pos != 0){
      Robot.m_elevatorSubsystem.setPos(m_pos);
    }else{
      Robot.m_elevatorSubsystem.set(-0.5);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    switch (m_stallCase){
      case checkEnd:
        boolean finished = false;
        if(m_up){
          finished = Robot.m_elevatorSubsystem.getPosRelativeHome() >= m_pos - k_error;
        }else{
          finished = Robot.m_elevatorSubsystem.getPosRelativeHome() <= m_pos + k_error;
        }
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
    System.out.println("stall Case: " + m_stallCase);
    if(isTimedOut()){
      System.out.println("isTimedout");
    }
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.Log("MoveElevatorPosCommand", 3, "End");
    Robot.m_elevatorSubsystem.stop();
  }
}
