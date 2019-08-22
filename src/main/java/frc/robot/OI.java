/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.Condition;
import frc.lib.ConditionCommand;
import frc.lib.PiCamera.TargetSelection;
import frc.robot.Navigator.CameraDirection;
import frc.robot.Navigator.FieldSide;
import frc.robot.Navigator.RocketSide;
import frc.robot.commands.LED.SetClimbLEDCommand;
import frc.robot.commands.arm.MoveArmOverrideCommand;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.StopArmConmand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.CameraAngleDriveCommand;
import frc.robot.commands.auto.DriveDropOffBackUpHatchCommand;
import frc.robot.commands.auto.DrivePickUpBackUpHatchCommand;
import frc.robot.commands.auto.DrivePickUpCargoCommand;
import frc.robot.commands.auto.LoadingZoneRocketCommand;
import frc.robot.commands.auto.RocketLoadingZoneCommand;
import frc.robot.commands.auto.autoTasks.ArmForwardCommand;
import frc.robot.commands.auto.autoTasks.ClimbCommand;
import frc.robot.commands.auto.autoTasks.DeployIntakeCommand;
import frc.robot.commands.auto.autoTasks.IntakeVerticalCommand;
import frc.robot.commands.auto.autoTasks.ReleasePistonsLevel2Command;
import frc.robot.commands.auto.autoTasks.RetractIntakeCommand;
import frc.robot.commands.auto.autoTasks.TeleopCargoShipPlacement;
import frc.robot.commands.climber.ActivatePistonsCommand;
import frc.robot.commands.climber.ClimbCommandPistons;
import frc.robot.commands.climber.Level2ActivatePistonsCommand;
import frc.robot.commands.climber.ReleasePistonsDoubleCommand;
import frc.robot.commands.drive.SpeedDriveCommand;
import frc.robot.commands.drive.TeleopCameraDriveCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.grabber.ToggleGrabberTeleopCommand;
import frc.robot.commands.intake.IntakeHomeCommand;
import frc.robot.commands.intake.MoveIntakeCommand;
import frc.robot.commands.intake.MoveIntakePosCommand;
import frc.robot.commands.intakeWheels.IntakeCargoCommand;
import frc.robot.commands.intakeWheels.OuttakeCargoCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.triggers.POVChangeTrigger;
import frc.robot.commands.intakeGroups.IntakeAllCommand;
import frc.robot.commands.intakeGroups.OuttakeAllCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
  public class HatchMode implements Condition{
    public boolean condition(){
      return hatchMode();
    }
  }
  Joystick m_driveStick = new Joystick(0);
  Joystick m_elevatorStick = new Joystick(1);
  Joystick m_backUpStick = new Joystick(2);
  Joystick m_testStick = new Joystick(3);

  // JoystickButton m_testLED = new JoystickButton(m_driveStick, 12);

  // JoystickButton m_powerDrive = new JoystickButton(m_driveStick, 5);
  // JoystickButton m_stallDrive = new JoystickButton(m_driveStick, 6);
  // JoystickButton m_hatchDrive = new JoystickButton(m_driveStick, 11);
  // JoystickButton m_cameraAngle = new JoystickButton(m_driveStick, 5);
  // JoystickButton m_climb = new JoystickButton(m_driveStick, 5);
  // JoystickButton m_release = new JoystickButton(m_driveStick, 3);
  // JoystickButton m_armUp = new JoystickButton(m_elevatorStick, 6);
  // JoystickButton m_armDown = new JoystickButton(m_elevatorStick, 4);
  JoystickButton m_speedDrive = new JoystickButton(m_testStick, 6);

  JoystickButton m_teleopAutoAlign = new JoystickButton(m_driveStick, 1);

  JoystickButton m_placeBack = new JoystickButton(m_driveStick, 5);
  JoystickButton m_placeFront = new JoystickButton(m_driveStick, 3);

  JoystickButton m_grabFront = new JoystickButton(m_driveStick, 6);
  JoystickButton m_grabBack = new JoystickButton(m_driveStick, 4);

  JoystickButton m_autoAlignLevel1Front = new JoystickButton(m_driveStick, 11);
  JoystickButton m_autoAlignLevel2Front = new JoystickButton(m_driveStick, 9);
  JoystickButton m_autoAlignLevel3Front = new JoystickButton(m_driveStick, 7);

  JoystickButton m_autoAlignLevel1Back = new JoystickButton(m_driveStick, 12);
  JoystickButton m_autoAlignLevel2Back = new JoystickButton(m_driveStick, 10);
  JoystickButton m_autoAlignLevel3Back = new JoystickButton(m_driveStick, 8);

  // JoystickButton m_testBtn = new JoystickButton(m_driveStick, 1);
  // JoystickButton m_testBtn2 = new JoystickButton(m_driveStick, 5);

  // JoystickButton m_stopArm = new JoystickButton(m_driveStick, 7);
  
  JoystickButton m_armUpPos = new JoystickButton(m_elevatorStick, 10);
  JoystickButton m_armBackPos = new JoystickButton(m_elevatorStick, 12);
  JoystickButton m_armForwardPos = new JoystickButton(m_elevatorStick, 8);
  // JoystickButton m_armIntakePos = new JoystickButton(m_elevatorStick, 5);
  
  JoystickButton m_grab = new JoystickButton(m_elevatorStick, 2);

  JoystickButton m_intakeHome = new JoystickButton(m_elevatorStick, 3);//convert to toggle on button 3
  JoystickButton m_intakeUpPos = new JoystickButton(m_elevatorStick, 4); //button 4
  // // JoystickButton m_intakeForwardPos = new JoystickButton(m_driveStick, 8);
  JoystickButton m_manipulateIntake = new JoystickButton(m_elevatorStick, 5);

  // JoystickButton m_intakeArmUp = new JoystickButton(m_driveStick, 6);
  // JoystickButton m_intakeArmDown = new JoystickButton(m_driveStick, 4);

  JoystickButton m_intake = new JoystickButton(m_elevatorStick, 1);
  // JoystickButton m_outtake = new JoystickButton(m_elevatorStick, 6);//switch to pov
  JoystickButton m_outtake = new JoystickButton(m_elevatorStick, 6);
  JoystickButton m_climbDrive = new JoystickButton(m_driveStick, 2);

  JoystickButton m_hatchLow = new JoystickButton(m_elevatorStick, 11);
  JoystickButton m_hatchMid = new JoystickButton(m_elevatorStick, 9); //Use throttle, hatch will be positive, cargo negative, then go to pos depending on that
  JoystickButton m_hatchHigh = new JoystickButton(m_elevatorStick, 7);

  JoystickButton m_climbBackUp = new JoystickButton(m_backUpStick, 1);
  JoystickButton m_releaseBackUp = new JoystickButton(m_backUpStick, 2);
  JoystickButton m_outtakeBackUp = new JoystickButton(m_backUpStick, 6);
  JoystickButton m_armOverrideUp = new JoystickButton(m_backUpStick, 10);
  JoystickButton m_armOverrideDown = new JoystickButton(m_backUpStick, 12);

  JoystickButton m_cargoShipFront = new JoystickButton(m_backUpStick, 9);
  JoystickButton m_cargoShipBack = new JoystickButton(m_backUpStick, 11);

  public OI(){

    // m_powerDrive.whileHeld(new PowerDriveCommand(0.75));
    m_speedDrive.whileHeld(new SpeedDriveCommand(1000));
    // m_hatchDrive.whenPressed(new DriveDropOffBackUpHatchCommand());
    // m_cameraAngle.whenPressed(new CameraAngleDriveCommand(1000, 2.0/12.0, CameraDirection.Front));
    
    m_teleopAutoAlign.whileHeld(new TeleopCameraDriveCommand());
    
    m_placeBack.whenPressed(new DriveDropOffBackUpHatchCommand(true, true, true, TargetSelection.auto));
    m_placeFront.whenPressed(new DriveDropOffBackUpHatchCommand(true, false, true, TargetSelection.auto));

    // m_grabFront.whenPressed(new DrivePickUpBackUpHatchCommand(true, false, true));
    // m_grabBack.whenPressed(new DrivePickUpBackUpHatchCommand(true, false, false));
    
    m_grabFront.whenPressed(new ConditionCommand(new HatchMode(), new DrivePickUpBackUpHatchCommand(true, false, true), new DrivePickUpCargoCommand(false, false, true)));
    m_grabBack.whenPressed(new ConditionCommand(new HatchMode(), new DrivePickUpBackUpHatchCommand(true, false, false), new DrivePickUpCargoCommand(false, false, false)));

    m_autoAlignLevel1Front.whenPressed(new DriveDropOffBackUpHatchCommand(true, false, true, ElevatorLevel.Level1));
    m_autoAlignLevel2Front.whenPressed(new DriveDropOffBackUpHatchCommand(true, false, true, ElevatorLevel.Level2));
    m_autoAlignLevel3Front.whenPressed(new DriveDropOffBackUpHatchCommand(true, false, true, ElevatorLevel.Level3));

    m_autoAlignLevel1Back.whenPressed(new DriveDropOffBackUpHatchCommand(true, true, true, ElevatorLevel.Level1));
    m_autoAlignLevel2Back.whenPressed(new DriveDropOffBackUpHatchCommand(true, true, true, ElevatorLevel.Level2));
    m_autoAlignLevel3Back.whenPressed(new DriveDropOffBackUpHatchCommand(true, true, true, ElevatorLevel.Level3));

    // m_testBtn.whenPressed(new ActivatePistonsCommand());
    // m_testBtn2.whenPressed(new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level1));

    // m_climb.whenPressed(new ActivatePistonsCommand());
    // m_release.whenPressed(new ReleaseCommand());

    // m_armUp.whileHeld(new MoveArmCommand(true));
    // m_armDown.whileHeld(new MoveArmCommand(false));

    m_armUpPos.whenActive(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    m_armBackPos.whenActive(new MoveArmPosCommand(ArmPos.Backward, RobotMap.k_finalRobot));
    // m_armForwardPos.whenActive(new MoveArmPosCommand(0, RobotMap.k_finalRobot));
    m_armForwardPos.whenActive(new ArmForwardCommand(true));

    // m_intakeHome.whenActive(new RetractIntakeCommand(0.25));
    // m_manipulateIntake.whenPressed(new DeployIntakeCommand(0));
    m_intakeUpPos.whenActive(new IntakeVerticalCommand());

    m_intakeHome.whenActive(new RetractIntakeCommand(1));
    m_manipulateIntake.whenPressed(new DeployIntakeCommand(0));

    // m_intakeArmUp.whileHeld(new MoveIntakeCommand(0.5));
    // m_intakeArmDown.whileHeld(new MoveIntakeCommand(-0.5));

    // m_hatchLow.whenPressed(new MoveElevatorPosCommand(ElevatorLevel.Level1, 0.5));
    // m_hatchMid.whenPressed(new MoveElevatorPosCommand(ElevatorLevel.Level2, 0.75));
    // m_hatchHigh.whenPressed(new MoveElevatorPosCommand(ElevatorLevel.Level3, 0.75));

    m_hatchLow.whenPressed(new MoveElevatorPosCommand(ElevatorLevel.Level1));
    m_hatchMid.whenPressed(new MoveElevatorPosCommand(ElevatorLevel.Level2));
    m_hatchHigh.whenPressed(new MoveElevatorPosCommand(ElevatorLevel.Level3));

    m_intake.whileHeld(new IntakeAllCommand());
    m_outtake.whileActive(new OuttakeAllCommand());

    m_climbDrive.whileActive(new IntakeCargoCommand(0.75));

    m_grab.toggleWhenPressed(new ToggleGrabberTeleopCommand());

    // m_stallDrive.whenPressed(new StallDriveCommand(0.15));

    // m_climb.whenPressed(new ClimbCommand());

    // m_intakeClimb.whileHeld(new IntakeClimbCommand(0.25));

    // m_stopArm.whenPressed(new StopArmConmand());

    // m_testLED.whenPressed(new SetClimbLEDCommand(true));

    m_climbBackUp.whenPressed(new ClimbCommand());
    m_releaseBackUp.whenPressed(new ReleasePistonsLevel2Command());
    m_outtakeBackUp.whileHeld(new OuttakeCargoCommand(1));
    m_armOverrideUp.whileHeld(new MoveArmOverrideCommand(0.25));
    m_armOverrideDown.whileHeld(new MoveArmOverrideCommand(-0.25));

    m_cargoShipFront.whenPressed(new TeleopCargoShipPlacement(true));
    m_cargoShipFront.whenPressed(new TeleopCargoShipPlacement(false));
  }

  public double getDriveX(){
    return m_driveStick.getX();
  }

  public double getDriveY(){
    return -m_driveStick.getY();
  }

  public double getDriveThrottle(){
    return -m_driveStick.getThrottle();
  }

  public double getElevatorY(){
    return -m_elevatorStick.getY();
  }

  public double getElevatorX(){
    return m_elevatorStick.getX();
  }

  public double getElevatorThrottle(){
    return -m_elevatorStick.getThrottle();
  }

  public boolean hatchMode(){
    return -m_elevatorStick.getThrottle() > 0;
  }

  public boolean getElevatorButton(int button){
    return m_elevatorStick.getRawButton(button);
  }
  
  public boolean getElevatorPOV(){
    return new POVChangeTrigger(m_elevatorStick).get();
  }
}
