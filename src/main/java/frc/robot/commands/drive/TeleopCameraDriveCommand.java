/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CSVWriter;
import frc.lib.PiCamera;
import frc.lib.CSVWriter.Field;
import frc.lib.PiCamera.TargetRegion;
import frc.lib.PiCamera.TargetSelection;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Navigator.CameraDirection;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.grabber.OpenGrabberCommand;
import frc.robotCore.Logger;

public class TeleopCameraDriveCommand extends Command {
  double k_pa = .006;
  double m_lastAngle;
  boolean m_endingDrive = false;
  Field[] k_fields = {
    new Field("Cur Angle", 'f'),
    new Field("Calcutaed Target Angle", 'f'),
    new Field("Offset Angle", 'f'),
    new Field("Pixels From Center", 'f')
  };
  CSVWriter m_writer = new CSVWriter("Camera Teleop Angle Drive", k_fields);

  private void setEndingDrive(){
    m_endingDrive = true;
    boolean forward = Robot.m_oi.getDriveThrottle() > 0;

    if(forward){
      new ArmForwardCommand(true).start();
    }else{
      new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot).start();
    }
  } 

  public TeleopCameraDriveCommand() {
    requires(Robot.m_driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_endingDrive = false;
    m_writer.start();

    if(Robot.m_elevatorSubsystem.getPosRelativeHome() < RobotMap.k_elevatorClearCamera && Robot.m_armSubsystem.hasHatch()){
      new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot).start();
    }
    Logger.Log("TeleopCameraDriveCommand", 3, "initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double correction = 0;
    boolean forward = Robot.m_oi.getDriveThrottle() > 0;
    CameraDirection cameraDirection;

    if(forward){
      cameraDirection = CameraDirection.Front;
    }else{
      cameraDirection = CameraDirection.Back;
    }

    if(!m_endingDrive){
      TargetRegion[] target = Robot.m_navigator.targetsClosestCenter(cameraDirection, TargetSelection.auto);

      double distAway = Robot.m_navigator.getCameraDistance(target, true, 160);

      Logger.Log("TeleopCameraDriveCommand", 3, String.format("distAway = %f", distAway));

      if(distAway == Double.MAX_VALUE){
        Logger.Log("TeleopCameraDriveCommand", 3, "setEndingDrive");
        correction = k_pa *(Robot.m_driveSubsystem.getAngle() - m_lastAngle);    
        setEndingDrive();
      }else{
        double ra = Robot.m_driveSubsystem.getAngle();
        double offsetAngle = Robot.m_navigator.getCameraAngleOffset(target, cameraDirection);

        // offsetAngle = ra + 94;

        correction = k_pa * offsetAngle;    
        
        m_lastAngle = Robot.m_driveSubsystem.getAngle();
        m_writer.write(ra, ra - offsetAngle, offsetAngle, PiCamera.getTargetCenter(target) - 320);
      }
    }else{
      double offset = Robot.m_driveSubsystem.getAngle() - m_lastAngle;
      correction = k_pa * offset;

      // if(Robot.m_armSubsystem.hasHatch()){
      //   new OpenGrabberCommand().start();
      // }
    }

    double power = Robot.m_oi.getDriveY();
    
    power = power * power * power;

    if(!forward){
      power = -power;
    }

    Robot.m_driveSubsystem.setPower(power + correction, power - correction);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveSubsystem.setPower(0, 0);
    m_writer.finish();
  }
}
