package frc.robot.commands.calibrate;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CSVWriter;
import frc.lib.CSVWriter.Field;
import frc.robot.Robot;
import frc.robot.Navigator.CameraDirection;
import frc.robot.Navigator.Target;
import frc.robot.PositionTracker.PositionContainer;

public class TestCameraDistanceBackCommand extends Command {
  Field[] k_fields = {
    new Field("Encoder X", 'f'),
    new Field("Encoder Y", 'f'),
    new Field("Camera X", 'f'),
    new Field("Camera Y", 'f'),
    new Field("Distance From Target", 'f'),
    new Field("Gyro", 'f'),
  };
  CSVWriter writer = new CSVWriter("Camera Test Back", k_fields);
  long lastWrite;
  
  public TestCameraDistanceBackCommand() {
    requires(Robot.m_driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.setGyro(90);
    Robot.m_driveSubsystem.setAngle(90);
    Robot.m_driveSubsystem.setXY(0,-4.5);
    lastWrite = System.currentTimeMillis();
    Robot.m_driveSubsystem.setCoastMode();
    writer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(System.currentTimeMillis() - lastWrite > 200){
      PositionContainer encoderPos = Robot.m_driveSubsystem.getPos();
      double targetWidth = Robot.m_navigator.getTargetPixelsApartCalibration(CameraDirection.Back);
      Target target = Robot.m_navigator.getCorrectTarget(CameraDirection.Back);
      PositionContainer cameraPos = Robot.m_navigator.getTargetXYDistance(targetWidth, target.m_pair, CameraDirection.Back);
  
      writer.write(encoderPos.x, encoderPos.y, cameraPos.x, cameraPos.y, Robot.m_navigator.getTargetDistance(targetWidth, target.m_pair, CameraDirection.Back), Robot.m_driveSubsystem.getAngle());
      lastWrite = System.currentTimeMillis();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    writer.finish();
    Robot.m_driveSubsystem.setBrakeMode();
  }
}
