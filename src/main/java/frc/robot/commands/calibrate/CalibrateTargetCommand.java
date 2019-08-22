package frc.robot.commands.calibrate;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CSVWriter;
import frc.lib.CSVWriter.Field;
import frc.robot.Robot;
import frc.robot.Navigator.CameraDirection;
import frc.robot.PositionTracker.PositionContainer;

public class CalibrateTargetCommand extends Command {
  double m_leftEncoder;
  double m_rightEncoder;

  private Field[] fields = {
    new Field("x", 'f'),
    new Field("y", 'f'),
    new Field("Target Apart", 'f'),
    new Field("Left Encoder", 'f'),
    new Field("Right Encoder", 'f')
  };

  CSVWriter frontWriter = new CSVWriter("Front Camera Target Calibration", fields);

  CSVWriter backWriter = new CSVWriter("Back Camera Target Calibration", fields);

  public CalibrateTargetCommand() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.setXY(0, 3.25);
    Robot.m_driveSubsystem.setAngle(270);
    Robot.m_driveSubsystem.setGyro(270);

    Robot.m_driveSubsystem.setCoastMode();

    frontWriter.start();
    backWriter.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    PositionContainer pos = Robot.m_driveSubsystem.getPos();

    double targetDistApartFront = Robot.m_navigator.getTargetPixelsApartCalibration(CameraDirection.Front);
    double targetDistApartBack = Robot.m_navigator.getTargetPixelsApartCalibration(CameraDirection.Back);

    frontWriter.write(pos.x, pos.y, targetDistApartFront, Robot.m_driveSubsystem.getLeftPos(), Robot.m_driveSubsystem.getRightPos());
    backWriter.write(pos.x, pos.y, targetDistApartBack, Robot.m_driveSubsystem.getLeftPos(), Robot.m_driveSubsystem.getRightPos());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    frontWriter.finish();
    backWriter.finish();
  }
}
