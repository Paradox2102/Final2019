package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CSVWriter;
import frc.lib.PiCamera;
import frc.lib.CSVWriter.Field;
import frc.lib.PiCamera.TargetRegion;
import frc.lib.PiCamera.TargetSelection;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Navigator.CameraDirection;
import frc.robotCore.Logger;

public class CameraAngleDriveCommand extends Command {
  double k_pa = 12  ;//15//For going forward10;
  double m_speed;
  double m_endingDistFromtarget;
  CameraDirection m_direction;
  double m_lastAngle;
  double m_distAway;
  double m_endingEncoder;
  boolean m_endingDrive = false;
  boolean m_forward;
  TargetSelection m_target;
  boolean m_earlyEnd;
  double m_endingPoint;
  Field[] k_fields = {
    new Field("Cur Angle", 'f'),
    new Field("Calcutaed Target Angle", 'f'),
    new Field("Offset Angle", 'f'),
    new Field("Pixels From Center", 'f')
  };
  CSVWriter m_writer = new CSVWriter("Camera Angle Drive", k_fields);

  public void setEndingDrive(){
    m_endingDrive = true;
    m_endingEncoder = (Robot.m_driveSubsystem.getLeftPosGrey() + Robot.m_driveSubsystem.getRightPosGrey())/2.0;
  }

  public CameraAngleDriveCommand(double speed, double distFromtarget, CameraDirection direction) {
    requires(Robot.m_driveSubsystem);

    m_speed = speed;
    m_endingDistFromtarget = distFromtarget;
    m_direction = direction;
    m_forward = true;
    m_target = TargetSelection.auto;
    m_earlyEnd = false;
    m_endingPoint = 0;
  }

  public CameraAngleDriveCommand(double speed, double distFromtarget, CameraDirection direction, boolean forward) {
    requires(Robot.m_driveSubsystem);

    m_speed = speed;
    m_endingDistFromtarget = distFromtarget;
    m_direction = direction;
    m_forward = forward;
    m_target = TargetSelection.auto;
    m_earlyEnd = false;
    m_endingPoint = 0;
  }

  public CameraAngleDriveCommand(double speed, double distFromtarget, CameraDirection direction, boolean forward, TargetSelection target, boolean earlyEnd, double endingPoint) {
    requires(Robot.m_driveSubsystem);

    m_speed = speed;
    m_endingDistFromtarget = distFromtarget;
    m_direction = direction;
    m_forward = forward;
    m_target = target;
    m_earlyEnd = earlyEnd;
    m_endingPoint = endingPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveSubsystem.setCameraCorrection(false);
    TargetRegion[] target = Robot.m_navigator.targetsClosestCenter(m_direction, m_target);
    double targetAngle = Robot.m_navigator.getTargetAngleSimple(target, m_direction);

    if(targetAngle >= Double.MAX_VALUE){
      end();
    }
    m_endingDrive = false;
    m_writer.start();
    // System.out.println("Camera Angle Drive init");
    Logger.Log("CameraAngleDriveCommand", 3, "initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double correction = 0;

    if(!m_endingDrive){
      TargetRegion[] target = Robot.m_navigator.targetsClosestCenter(m_direction, m_target);

      double distAway = Robot.m_navigator.getCameraDistance(target, m_earlyEnd, m_endingPoint);

      Logger.Log("CameraAngleDriveCommand", 3, String.format("distAway = %f", distAway));

      if(distAway == Double.MAX_VALUE){
        Logger.Log("CameraAngleDriveCommand", 3, "setEndingDrive");
        setEndingDrive();
      }else{
        double ra = Robot.m_driveSubsystem.getAngle();
        double offsetAngle = Robot.m_navigator.getCameraAngleOffset(target, m_direction);

        // offsetAngle = ra + 94;

        correction = k_pa * offsetAngle;
        
        
        m_lastAngle = offsetAngle;
        m_distAway = distAway - RobotMap.k_cameraDistToBumpers;

        m_writer.write(ra, ra - offsetAngle, offsetAngle, PiCamera.getTargetCenter(target) - 320);
      }
    }else{
      correction = k_pa * m_lastAngle;
    }
    if(m_forward){
      Robot.m_driveSubsystem.setSpeedSlow(m_speed + correction, m_speed - correction);
    }else{
      Robot.m_driveSubsystem.setSpeedSlow(-m_speed + correction, -m_speed - correction);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(m_endingDrive){
      return true;
      // return (m_distAway - distTraveled) < m_endingDistFromtarget;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveSubsystem.stop();

    m_writer.finish();
    // System.out.println("Camera Angle Drive end");
    Logger.Log("CameraAngleDriveCommand", 3, "End");
  }
}