package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.PiCamera.TargetSelection;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Navigator.CameraDirection;
import frc.robot.Navigator.CargoSide;
import frc.robot.Navigator.FieldSide;
import frc.robot.Navigator.RocketSide;
import frc.robot.PositionTracker.PositionContainer;
import frc.robot.commands.arm.MoveArmPosCommand;
import frc.robot.commands.arm.MoveArmPosCommand.ArmPos;
import frc.robot.commands.auto.AngledLeftLoadingZoneLeftSideCargoShipCommand;
import frc.robot.commands.auto.CreatePathCommand;
import frc.robot.commands.auto.DoNothingCommand;
import frc.robot.commands.auto.DriveDropOffBackUpHatchCommand;
import frc.robot.commands.auto.DrivePickUpBackUpHatchCommand;
import frc.robot.commands.auto.PlatformRocketCommand;
import frc.robot.commands.auto.RightFrontRightCargoShipCommand;
import frc.robot.commands.auto.CompletedPaths.CenterFrontCargoShipLoadingZoneFrontCargoShipCommand;
import frc.robot.commands.auto.CompletedPaths.CenterFrontCargoShipLoadingZoneFrontRocketCommand;
import frc.robot.commands.auto.CompletedPaths.LeftFrontLeftCargoShipLoadingZoneSideCargoShipCommand;
import frc.robot.commands.auto.CompletedPaths.PlatformFrontCargoShipLoadingZoneRocketCommand;
import frc.robot.commands.auto.CompletedPaths.PlatformFrontCargoShipLoadingZoneSideCargoShipCommand;
import frc.robot.commands.auto.CompletedPaths.PlatformSideCargoShipLoadingZoneFrontCargoShipCommand;
import frc.robot.commands.auto.CompletedPaths.RightFrontRightCargoShipLoadingZoneSideCargoShipCommand;
import frc.robot.commands.drive.TeleopCameraDriveCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.GrabberWheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeWheelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robotCore.Logger;

public class Robot extends TimedRobot {
  public static DriveSubsystem m_driveSubsystem;
  public static ElevatorSubsystem m_elevatorSubsystem;
  public static ClimberSubsystem m_climberSubsystem;
  public static IntakeSubsystem m_intakeSubsystem;
  public static ArmSubsystem m_armSubsystem;
  public static GrabberSubsystem m_grabberSubsystem;
  public static GrabberWheelSubsystem m_grabberWheelSubsystem;
  public static IntakeWheelSubsystem m_intakeWheelsSubsystem;
  public static LEDSubsystem m_LEDSubsystem1;
  public static LEDSubsystem m_LEDSubsystem2;
  public static Subsystem m_ledControlSubsystem;
  public static Navigator m_navigator;
  public static OI m_oi;

  CameraServer cameras;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    Logger.SetLogFile("CommandLog");
    m_driveSubsystem = new DriveSubsystem();
    m_elevatorSubsystem = new ElevatorSubsystem();
    m_climberSubsystem = new ClimberSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_armSubsystem = new ArmSubsystem();
    m_grabberSubsystem = new GrabberSubsystem();
    m_grabberWheelSubsystem = new GrabberWheelSubsystem();
    m_intakeWheelsSubsystem = new IntakeWheelSubsystem();
    m_LEDSubsystem1 = new LEDSubsystem(9, 82);
    m_LEDSubsystem2 = new LEDSubsystem(8, 82);
    m_ledControlSubsystem = new Subsystem(){
    
      @Override
      protected void initDefaultCommand() {
        
      }
    };
    m_navigator = new Navigator();
    m_oi = new OI();

    //Cameras
    cameras = CameraServer.getInstance();
		cameras.startAutomaticCapture(0);
    cameras.startAutomaticCapture(1);

    m_LEDSubsystem1.setClimbing(false);
    m_LEDSubsystem2.setClimbing(false);
    
    // chooser.addOption("My Auto", new MyAutoCommand());
    m_chooser.setDefaultOption("Do Nothing", new DoNothingCommand());
    // // m_chooser.addOption("Calibrate Ticks Per Foot", new CalibrateTicksPerFootCommand());
    // m_chooser.addOption("Drive Straight 12 ft", new CreatePathCommand(new Waypoint[] {
		// 	new Waypoint(0, 0, Math.toRadians(90), 0, 0, 0),
		// 	new Waypoint(0, 20, Math.toRadians(90), 0, 0, 0)
    // }, m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, false, true, false));
    // // m_chooser.addOption("Print Path", new PrintPathCommand(new Waypoint[] {
		// // 	new Waypoint(0, 0, Math.toRadians(90), 0, 0, 0),
		// // 	new Waypoint(0, 10, Math.toRadians(90), 0, 0, 0)
    // // }, m_driveSubsystem.m_pursuitFollower.m_pathConfigFast));
    // m_chooser.addOption("Auto Center Cargo Front Right", new CenterFrontCargoShipCommand(FieldSide.Right, false));
    // m_chooser.addOption("Auto Center Cargo Front Left", new CenterFrontCargoShipCommand(FieldSide.Left, false));
    // m_chooser.addOption("Auto Right Cargo Front Right", new RightFrontRightCargoShipCommand());
    // m_chooser.addOption("Auto Left Front Cargo Ship to Right Loading Zone", new FrontCargoShipRightLoadingZoneCommand(FieldSide.Left, false));
    // m_chooser.addOption("Auto Right Loading Zone to Front Right Cargo Ship", new LoadingZoneFrontCargoShipCommand(FieldSide.Right, false));
    // m_chooser.addOption("Auto Right Loading zone to close Side Cargo Ship", new RightLoadingZoneSideCargoShipCommand(CargoSide.Close));
    // m_chooser.addOption("Calibrate Camera", new CalibrateTargetCommand());
    // // m_chooser.addOption("Stall Command", new StallDriveCommand(0.15));
    // // m_chooser.addOption("Position Tester", new PosTesterCommand());
    // // m_chooser.addOption("Camera Test Front", new TestCameraDistanceFrontCommand());
    // // m_chooser.addOption("Camera Test Back", new TestCameraDistanceBackCommand());
    // // m_chooser.addOption("Camera Error Test", new CameraErrorTestCommand());
    // // m_chooser.addOption("Camera Angle Drive", new CameraAngleDriveCommand(1000, CameraDirection.Front));
    // m_chooser.addOption("Auto Right Cargo To Side Cargo", new PlatformFrontCargoShipLoadingZoneSideCargoShipCommand(FieldSide.Right, CargoSide.Close));//Center Front RIght CS Right LZ Right Close Side CS
    // m_chooser.addOption("Auto Center Front Left CS Right LZ Right CS", new CenterFrontCargoShipLoadingZoneFrontCargoShipCommand(FieldSide.Right));
    m_chooser.addOption("Auto Right Rocket 2 Hatches", new PlatformRocketCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level3));//Right Platform to Close RS
    m_chooser.addOption("Auto Left Rocket 2 Hatches", new PlatformRocketCommand(FieldSide.Left, RocketSide.Close, ElevatorLevel.Level3));//Right Platform to Close RS

    
    m_chooser.addOption("Auto Center(Starts aligned Right Port Goes Right) Front CargoShip 2 Hatches", new CenterFrontCargoShipLoadingZoneFrontCargoShipCommand(FieldSide.Right));
    m_chooser.addOption("Auto Center(Starts aligned Left Port Goes Left) Front CargoShip 2 Hatches", new CenterFrontCargoShipLoadingZoneFrontCargoShipCommand(FieldSide.Left));

    m_chooser.addOption("Auto Center(Starts aligned Right Port Goes Right) Front CargoShip Front Rocket Level 2", new CenterFrontCargoShipLoadingZoneFrontRocketCommand(FieldSide.Right, ElevatorLevel.Level2));
    m_chooser.addOption("Auto Center(Starts aligned Right Port Goes Right) Front CargoShip Front Rocket Level 3", new CenterFrontCargoShipLoadingZoneFrontRocketCommand(FieldSide.Right, ElevatorLevel.Level3));

    m_chooser.addOption("Auto Center(Starts aligned Left Port Goes Left) Front CargoShip Front Rocket Level 2", new CenterFrontCargoShipLoadingZoneFrontRocketCommand(FieldSide.Left, ElevatorLevel.Level2));
    m_chooser.addOption("Auto Center(Starts aligned Left Port Goes Left) Front CargoShip Front Rocket Level 3", new CenterFrontCargoShipLoadingZoneFrontRocketCommand(FieldSide.Left, ElevatorLevel.Level3));
    
    m_chooser.addOption("Auto Right Front CargoShip Front Rocket Level 2", new PlatformFrontCargoShipLoadingZoneRocketCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level2));
    m_chooser.addOption("Auto Left Front Cargo Ship Side Cargo Ship", new LeftFrontLeftCargoShipLoadingZoneSideCargoShipCommand());
    m_chooser.addOption("Auto Right Front Cargo Ship Side Cargo Ship", new RightFrontRightCargoShipLoadingZoneSideCargoShipCommand());
    // m_chooser.addOption("Temp Loading", new CommandGroup(){
    //   {
    //     addParallel(new MoveArmPosCommand(ArmPos.HatchCameraAvoid, RobotMap.k_finalRobot));
    //     // addSequential(new AngledLeftLoadingZoneLeftSideCargoShipCommand());
    //     // addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, TargetSelection.right, false));
    //   }
    // });
    // final Waypoint[] leftLoadingZoneToFrontRightCargoShipPath = {
    //   new Waypoint(-11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90),6.5,3,0),
    //   // new Waypoint(-6.5, 9.5, Math.toRadians(20), 2, 4, 0),
    //   new Waypoint(-2.5, 13.5, Math.toRadians(55), 3.3)
    //   // new Waypoint(11.36, RobotMap.k_chassisLength/2.0, Math.toRadians(90), 2, 2, 0),
    //   // new Waypoint(6.5, 8.5, Math.toRadians(155), 2, 5, 0),
    //   // new Waypoint(-0.9, 14.5, Math.toRadians(90), 3.3)
    // };

    // final Waypoint[] leftSideToLeftCloseRocketPath = {
    //   // new Waypoint(5.4 - RobotMap.k_chassisWidth/2.0, 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 4, 3, 6),
    //   // new Waypoint(8.7, 13, Math.toRadians(60), 0, 0, 3.3)
    //   new Waypoint(-(5.4 - RobotMap.k_chassisWidth/2.0), 4 + RobotMap.k_chassisLength/2.0, Math.toRadians(90), 3, 3, 6),
    //   new Waypoint(-8.2, 12.13, Math.toRadians(120), 0, 0, 0)
    // };

    // m_chooser.addOption("Matt Path Test", new CommandGroup(){
    //   {
    //     addSequential(new MoveArmPosCommand(ArmPos.Up, RobotMap.k_finalRobot));
    //     addSequential(new CreatePathCommand(leftSideToLeftCloseRocketPath, Robot.m_driveSubsystem.m_pursuitFollower.m_pathConfigFast, true, true, false, false, true));
    //     // addSequential(new DriveDropOffBackUpHatchCommand(false, true, false, 1000));
    //   }
    // });
        // m_chooser.addOption("Auto Right Front CargoShip Back Rocket Level 2", new PlatformFrontCargoShipLoadingZoneRocketCommand(FieldSide.Right, RocketSide.Far, ElevatorLevel.Level2));
    // m_chooser.addOption("Pick Up hatch and drive back", new DrivePickUpBackUpHatchCommand(true));
    // m_chooser.addOption("Drop off hatch and drive back", new DriveDropOffBackUpHatchCommand(true));

    m_driveSubsystem.setCameraCorrection(false);

    m_climberSubsystem.set(Value.kReverse);
    
    m_navigator.initNetwork();

    m_armSubsystem.setStall(false);
    m_intakeSubsystem.setStall(false);

    SmartDashboard.putData("Auto mode", m_chooser);  
  }

  @Override
  public void robotPeriodic() {
    PositionContainer pos = m_driveSubsystem.getPos();
    m_navigator.sendPositionData(pos);
    m_navigator.sendTimeRemaining();

    m_navigator.sendCameraData(CameraDirection.Front);
    m_navigator.sendCameraData(CameraDirection.Back);

    m_navigator.sendCameraPos(CameraDirection.Front);
    m_navigator.sendCameraPos(CameraDirection.Back);

    if(!m_elevatorSubsystem.getRevLimit()){
      m_elevatorSubsystem.homeEncoder();
    }

    if(m_intakeSubsystem.getLimit()){
      m_intakeSubsystem.homeEncoder();
    }

    if(m_armSubsystem.getPos() < 10){
      m_armSubsystem.disableMotor();
    }

    if(Math.abs(m_oi.getDriveY()) > 0.15){
      if(m_driveSubsystem.getCurrentCommandName() != m_driveSubsystem.getDefaultCommandName() && !m_driveSubsystem.getCurrentCommandName().equals("TeleopCameraDriveCommand")){
        System.out.println(m_driveSubsystem.getCurrentCommandName());
        m_driveSubsystem.getCurrentCommand().cancel();
      }
    }

    if(Math.abs(m_oi.getElevatorX()) > 0.15){
      if(m_elevatorSubsystem.getCurrentCommandName() != m_elevatorSubsystem.getDefaultCommandName()){
        if(m_elevatorSubsystem.getCurrentCommand() != null){
          m_elevatorSubsystem.getCurrentCommand().cancel();
          m_elevatorSubsystem.disablePid();
        }
        
        if(m_intakeSubsystem.getCurrentCommand() != null){
          m_intakeSubsystem.getCurrentCommand().cancel();
          m_intakeSubsystem.disablePid();
        }
        
        if(m_armSubsystem.getCurrentCommand() != null){
          m_armSubsystem.getCurrentCommand().cancel();
          m_armSubsystem.disablePid();
        }

        if(m_grabberWheelSubsystem.getCurrentCommand() != null){
          m_grabberWheelSubsystem.getCurrentCommand().cancel();
        }

        if(m_grabberSubsystem.getCurrentCommand() != null){
          m_grabberSubsystem.getCurrentCommand().cancel();
        }

        if(m_intakeWheelsSubsystem.getCurrentCommand() != null){
          m_intakeWheelsSubsystem.getCurrentCommand().cancel();
        }
      }
    }

    SmartDashboard.putNumber("Arm Pos", m_armSubsystem.getPos());

    SmartDashboard.putNumber("elevator Pos", m_elevatorSubsystem.getPosRelativeHome());
    SmartDashboard.putNumber("Arm Angle", m_armSubsystem.getAngle());
    SmartDashboard.putNumber("intake Angle", m_intakeSubsystem.getAngle());
    SmartDashboard.putBoolean("Has Hatch", m_armSubsystem.hasHatch());
    SmartDashboard.putNumber("Intake Pos", m_intakeSubsystem.getPosRelativeHome());
    SmartDashboard.putNumber("Gyro", m_driveSubsystem.getAngle());
    SmartDashboard.putNumber("Left Encoder", m_driveSubsystem.getLeftPosGrey());
    SmartDashboard.putNumber("Right Encoder", m_driveSubsystem.getRightPosGrey());
    SmartDashboard.putNumber("Elevator Pos", m_elevatorSubsystem.getPosRelativeHome());
    SmartDashboard.putBoolean("Elevator Limit", m_elevatorSubsystem.getRevLimit());
    SmartDashboard.putData(m_driveSubsystem);
    SmartDashboard.putNumber("Elevator Current", m_elevatorSubsystem.getAmp());

    SmartDashboard.putNumber("Pitch", m_driveSubsystem.getPitch());

    m_armSubsystem.checkLimit();
  }

  @Override
  public void disabledInit() {
    m_driveSubsystem.setCoastMode();
    m_driveSubsystem.stopPosUpdate();
    m_climberSubsystem.release();
    m_elevatorSubsystem.brake();
    m_intakeSubsystem.brakeMode();

    m_armSubsystem.stopWriter();

    m_armSubsystem.setStall(false);
    m_intakeSubsystem.setStall(false);

    // m_navigator.finishCameraWriter();

    m_intakeSubsystem.disablePid();
    m_armSubsystem.disablePid();
  }

  @Override
  public void disabledPeriodic() {
    // PositionContainer pos = m_driveSubsystem.getPos();
    // SmartDashboard.putNumber("Left Encoder Pos Grey", m_driveSubsystem.getLeftPosGrey());
    // SmartDashboard.putNumber("Right Encoder Pos Grey", m_driveSubsystem.getRightPosGrey());
    // SmartDashboard.putNumber("Gyro", m_driveSubsystem.getAngle());
    // SmartDashboard.putString("x", String.format("%.2f", pos.x));
    // SmartDashboard.putString("y", String.format("%.2f", pos.y));
    // SmartDashboard.putNumber("Distance from target", m_navigator.getTargetDistance()*12.0);
    
    // PositionContainer centerPos = m_navigator.getTargetXYDistance();
    // SmartDashboard.putString("X from Target", String.format("%.2f", centerPos.x));
    // SmartDashboard.putString("Y from Target", String.format("%.2f", centerPos.y));
    // SmartDashboard.putNumber("Pixels Apart", m_navigator.getTargetPixelsApartCalibration());

    SmartDashboard.putNumber("Arm Pos", m_armSubsystem.getPos());
    // SmartDashboard.putNumber("Intake Pos", m_intakeSubsystem.getPos());
    // SmartDashboard.putNumber("Intake Relative Pos", m_intakeSubsystem.getPosRelativeHome());
    // SmartDashboard.putBoolean("Hatch Limit Switch", m_armSubsystem.hasHatch());

    // SmartDashboard.putNumber("Pitch", m_driveSubsystem.getPitch());

    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();//new PlatformFrontCargoShipLoadingZoneSideCargoShipCommand(FieldSide.Right, CargoSide.Close);

    m_driveSubsystem.setBrakeMode();

    // m_driveSubsystem.setBrakeMode();
    // m_driveSubsystem.setSpeed(100, 100);
    // for (int i = 0 ; i < 50 ; i++)
    // {
    //   System.out.println(String.format("+left=%f,right=%f", m_driveSubsystem.getLeftPos(), m_driveSubsystem.getRightPos()));
    //   try {
    //     Thread.sleep(10);
    //   } catch (InterruptedException e) {
    //     // TODO Auto-generated catch block
    //     e.printStackTrace();
    //   }
    // }
    // m_driveSubsystem.stop();
    m_driveSubsystem.setCameraCorrection(true);
    // m_navigator.setTargetPosFront(new PositionContainer(0.9, 18.35));
    m_driveSubsystem.setXY(0, 0);

    // m_driveSubsystem.setGyroSPI(90);
    m_driveSubsystem.setGyro(-90);
    m_driveSubsystem.setAngle(-90);

    m_driveSubsystem.startPosUpdate();

    m_intakeSubsystem.brakeMode();

    m_grabberSubsystem.open();

    m_armSubsystem.setStall(false);
    m_intakeSubsystem.setStall(false);

    // m_navigator.startCameraWriter();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    
    PositionContainer pos = m_driveSubsystem.getPos();
    // SmartDashboard.putNumber("Gyro SPI Raw", m_driveSubsystem.getRawAngleSPI());
    // SmartDashboard.putNumber("Gyro SPI", m_driveSubsystem.getAngleSPI());
    SmartDashboard.putNumber("Gyro", m_driveSubsystem.getAngle());
    SmartDashboard.putString("x", String.format("%.2f", pos.x));
    SmartDashboard.putString("y", String.format("%.2f", pos.y));
    SmartDashboard.putNumber("Left Encoder", m_driveSubsystem.getLeftPosGrey());
    SmartDashboard.putNumber("Right Encoder", m_driveSubsystem.getRightPosGrey());
    // TargetRegion[] target = m_navigator.targetsClosestCenter(CameraDirection.Front);
    // SmartDashboard.putNumber("Target Angle", m_navigator.getTargetAngleSimple(target, CameraDirection.Front));
    SmartDashboard.putBoolean("Hatch Limit Switch", m_armSubsystem.hasHatch());
    Logger.Log("Robot Drive Subsystem", 3, String.format("Command: %s", Robot.m_driveSubsystem.getCurrentCommandName()));
  }

  @Override
  public void teleopInit() {
    m_driveSubsystem.setBrakeMode();
    m_intakeSubsystem.brakeMode();

    // m_driveSubsystem.setGyroSPI(90);
    // m_driveSubsystem.setGyro(90);
    // m_driveSubsystem.setAngle(90);

    m_driveSubsystem.setCameraCorrection(false);
   
    //TODO: Only set these at auto init
    m_armSubsystem.setStall(false);
    m_intakeSubsystem.setStall(false);
    m_intakeSubsystem.disablePid();
    m_armSubsystem.disablePid();

    // m_driveSubsystem.setXY(73.0/12.0, 21.5);//0, 8 + RobotMap.k_chassisLength/2.0);
    m_driveSubsystem.startPosUpdate();

    m_armSubsystem.setLastPos();
    m_armSubsystem.setStall(true);

    m_armSubsystem.write();
 
    if (m_autonomousCommand != null) {
      // m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putNumber("Left Encoder Pos Enabled", m_driveSubsystem.getLeftPos());
    // SmartDashboard.putNumber("Gyro", Navigator.normalizeAngleDeg(m_driveSubsystem.getAngle()));
    // SmartDashboard.putString("x", String.format("%.2f", pos.x));
    // SmartDashboard.putString("y", String.format("%.2f", pos.y));
    // SmartDashboard.putNumber("Elevator Relative Pos", m_elevatorSubsystem.getPosRelativeHome());
    // SmartDashboard.putNumber("Arm Relative Pos", m_armSubsystem.getAngle());
    // SmartDashboard.putNumber("Intake Pos", m_intakeSubsystem.getPos());
    // SmartDashboard.putNumber("Intake Relative Pos", m_intakeSubsystem.getPosRelativeHome());
    // SmartDashboard.putBoolean("Hatch Limit Switch", m_armSubsystem.hasHatch());
    // SmartDashboard.putNumber("Accelometer", m_driveSubsystem.getAccelY());
  //TODO: Delete After Testing
    // TargetRegion[] target =  m_navigator.getCorrectTargetFront();

    // PositionContainer cameraXY = m_navigator.getTargetXYDistanceFront(m_navigator.getTargetPixelsApartFront(target), target);
    // SmartDashboard.putNumber("X from Target Front", cameraXY.x);
    // SmartDashboard.putNumber("Y from Target Front", cameraXY.y);

    // int[] lineData = Robot.m_navigator.lineFollower();
    // for(int i=0; i<8; i++){
    //   System.out.print(String.format("Id: %d, color: %d ", i, lineData[i]));
    // }
    // System.out.println("");
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }
}
