/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //Motors
  public static int k_leftDrive = 1;
  public static int k_leftDriveFollower = 3;
  public static int k_rightDrive = 2;
  public static int k_rightDriveFollower = 4;

  public static int k_elevator = 5;

  public static int k_arm = 6;
  public static int k_grabberWheels = 7;
  
  public static int k_intakeArm = 8;

  public static int k_intakeWheels = 9;
  public static int k_intakeWheelsFollower = 10;

  //Pneumatics
  public static int k_brake =5;
  public static int k_grabberForward = 6;
  public static int k_grabberReverse = 7;
  public static int k_climber = 4;

  public static int k_climberForward = 0;
  public static int k_climberBackward = 1;

  //Encoders
  public static int k_leftDriveEncoderA = 3;
  public static int k_leftDriveEncoderB = 4;
  public static int k_rightDriveEncoderA = 1;
  public static int k_rightDriveEncoderB = 2;

  public static int k_elevatorEncoderA = 5;
  public static int k_elevatorEncoderB = 6;

  public static int k_intakeEncoderA = 7;
  public static int k_intakeEncoderB = 8;


  public static int k_hatchLimitSwitch = 0;

  //Limit Switches
  public static int k_elevatorLimit = 9;

  //Constants
  public static int k_CTREimeout = 10;
  
	public static double fixedTargetApart = 8.0/12.0;
	public static double targetConstant = 436.82;//453.77;//415.6986;//D*l
	public static double targetPower = 1;
  public static double k_targetHeight = 2.45;//2.1;//2.07;
  
  public static double k_cameraFrontAngleOffset = 2.53;
  public static double k_cameraBackAngleOffset = 0;

  public static int whiteColor = 175;
  
  public static double k_stallPower = 100;

  public static double k_chassisLength = 3.125;//2.66;
  public static  double k_chassisWidth = 2.708;//26.75/12.0; 

  public static double k_frontCSError = 5.0; //12.0;
  public static double k_sideCSError = 3.0;

  public static double k_leftLoadingZoneKludge = 0;

  public static double k_cameraToCenterDist = 15.5/12.0;
  public static double k_cameraDistToBumpers = 5.0/12.0;

  public static double k_armStallPower = 0.1;
  public static int k_armUpPos = 570;
  public static double k_armIntakeAngle = -56;//(960);
  public static double k_armFeederAngleForward = 58;// 494;
  public static double k_armFeederAngleBackward = 120;// 494;
  public static double k_cargoShipAngleFront = -27;
  public static double k_cargoShipAngleBack = 208;
  public static double k_cargoRocketAngleFront = -18;
  public static double k_cargoRocketAngleBack = 195;
  public static double k_hatchCameraAvoid = 140;

  public static double k_elevatorStallPower = 0.2;

  public static int k_rocketHatchLevel1 = 0;
  public static int k_rocketHatchLevel2 = 7100;
  public static int k_rocketHatchLevel3 = 13900;

  public static int k_intakeClimbPos = 2710;
  public static int k_intakeClimbLevel2Pos = 10;

  public static int k_rocketCargoLevel1 = 4000;
  public static int k_rocketCargoLevel2 = 11000;//9250;
  public static int k_rocketCargoLevel3 = 15500;

  public static int k_elevatorIntakeAvoidHeight = 6500;
  public static int k_climbHeight = 6500;
  public static int k_elevatorIntakeHeight = 3250;
  public static int k_elevatorArmAvoidHeight = 5000;

  public static int k_elevatorFeederPlacement = 400;
  public static int k_elevatorGrabHeight = 800;
  public static int k_elevatorCargoShip = 7700;
  public static int k_elevatorClearCamera = 1300;

  public static int k_intakeUpPos = 90;

  public static int k_intakeHorizontal = 3032;

  public static boolean k_finalRobot = false;
}
