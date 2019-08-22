/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.lib.PIDController;
import frc.lib.SPIGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.lib.CSVWriter;
import frc.lib.CSVWriter.Field;
import frc.pathfinder.Pathfinder.Path;
import frc.robot.PositionTracker;
import frc.robot.PurePursuit;
import frc.robot.RobotMap;
import frc.robot.Sensor;
import frc.robot.SparkMaxWrapperVel;
import frc.robot.Navigator.CameraDirection;
import frc.robot.PositionTracker.PositionContainer;
import frc.robot.PurePursuit.VelocityContainer;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robotCore.Logger;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  CANSparkMax m_leftDrive = new CANSparkMax(RobotMap.k_leftDrive, MotorType.kBrushless);
  CANSparkMax m_leftDriveFollower = new CANSparkMax(RobotMap.k_leftDriveFollower, MotorType.kBrushless);
  CANSparkMax m_rightDrive = new CANSparkMax(RobotMap.k_rightDrive, MotorType.kBrushless);
  CANSparkMax m_rightDriveFollower = new CANSparkMax(RobotMap.k_rightDriveFollower, MotorType.kBrushless);

  Accelerometer m_accel = new BuiltInAccelerometer();

  Encoder m_leftGreyEncoder;
  Encoder m_rightGreyEncoder;

  CANPIDController m_leftPIDSpark = new CANPIDController(m_leftDrive);
  CANPIDController m_rightPIDSpark = new CANPIDController(m_rightDrive);

  PIDController m_leftPIDController;
  PIDController m_rightPIDController;

  SparkMaxWrapperVel m_leftWrapper;
  SparkMaxWrapperVel m_rightWrapper;

  CANEncoder m_leftSparkEncoder;
  CANEncoder m_rightSparkEncoder;

  double k_ticksFootSpark = 5.1142;
  double k_ticksFootGrey = 694.95;

  //Normal PID Settings
  double k_fLeft = 0.00018; //0.000178;//0.000167
  double k_pLeft = 0.00022 * 1.5; //0.0018;//0.0001;
  double k_iLeft = 0.000015;

  double k_fRight = 0.00018; //0.00018;//0.000167
  double k_pRight = 0.00011 * 1.5; //0.001;//14;//0.0001;
  double k_iRight = 0.000015;

  //Slow PID settings
  double k_fLeftSlow = 0.00018;//0.000167
  double k_pLeftSlow = 0.00011;//0.0001;
  double k_iLeftSlow = 0.000015;

  double k_fRightSlow = 0.00018;//0.000167
  double k_pRightSlow = 0.000055;//14;//0.0001;
  double k_iRightSlow = 0.000015;

  double k_iRange = 100;
  double k_minOutput = -1;
  double k_maxOutput = 1;

  private PigeonIMU m_gyro = new PigeonIMU(0);

  private SPIGyro m_gyroSPI = new SPIGyro(SPI.Port.kOnboardCS0);
  
  private final long k_mpPeriod = 20; //Milliseconds

  private VelocityContainer m_velContainer;

  private PositionTracker m_posTracker;

  public final PurePursuit m_pursuitFollower;
	
  private Sensor m_sensors;
  // private SensorSPI m_sensorsSPI;
  
  private final FollowThread followThread = new FollowThread();

  private boolean m_reversed = false;

  private boolean m_normalPid = true;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopDriveCommand());
  }

  public DriveSubsystem(){
    m_leftDrive.setInverted(true);

    m_leftDriveFollower.follow(m_leftDrive);
    m_rightDriveFollower.follow(m_rightDrive);

    // m_leftDrive.setSmartCurrentLimit(40);
    // m_rightDrive.setSmartCurrentLimit(40);
    // m_leftDriveFollower.setSmartCurrentLimit(40);
    // m_rightDriveFollower.setSmartCurrentLimit(40);

    m_leftPIDSpark = m_leftDrive.getPIDController();
    m_rightPIDSpark = m_rightDrive.getPIDController();

    m_leftPIDSpark.setFF(k_fLeft);
    m_rightPIDSpark.setFF(k_fRight);
    m_leftPIDSpark.setP(k_pLeft);
    m_rightPIDSpark.setP(k_pRight);
    m_leftPIDSpark.setIZone(k_iRange);
    m_rightPIDSpark.setIZone(k_iRange);

    m_leftPIDSpark.setOutputRange(k_minOutput, k_maxOutput);
    m_rightPIDSpark.setOutputRange(k_minOutput, k_maxOutput);

    m_leftSparkEncoder = m_leftDrive.getEncoder();
    m_rightSparkEncoder = m_rightDriveFollower.getEncoder();

    m_leftWrapper = new SparkMaxWrapperVel(m_leftDrive, m_leftSparkEncoder, false);
    m_rightWrapper = new SparkMaxWrapperVel(m_rightDrive, m_rightSparkEncoder, false);

    // m_leftPIDController = new PIDController(k_p, 0, 0, k_f, m_leftWrapper, m_leftWrapper);
    // m_rightPIDController = new PIDController(k_p, 0, 0, k_f, m_rightWrapper, m_rightWrapper);
    m_leftPIDController = new PIDController(k_iLeft, 0, k_pLeft, k_fLeft, m_leftWrapper, m_leftWrapper, 0.02);
    m_leftPIDController.setIRange(k_iRange);
    m_rightPIDController = new PIDController(k_iRight, 0, k_pRight, k_fRight, m_rightWrapper, m_rightWrapper, 0.02);
    m_rightPIDController.setIRange(k_iRange);

    m_leftGreyEncoder = new Encoder(RobotMap.k_leftDriveEncoderA, RobotMap.k_leftDriveEncoderB, false, EncodingType.k4X);
    m_rightGreyEncoder = new Encoder(RobotMap.k_rightDriveEncoderA, RobotMap.k_rightDriveEncoderB, true, EncodingType.k4X);

    m_sensors = new Sensor(m_leftSparkEncoder, m_rightSparkEncoder, m_leftGreyEncoder, m_rightGreyEncoder, m_gyro);
    // m_sensorsSPI = new SensorSPI(m_leftSparkEncoder, m_rightSparkEncoder, m_leftGreyEncoder, m_rightGreyEncoder, m_gyroSPI);

    m_posTracker = new PositionTracker(0, 0, k_ticksFootGrey, false, m_sensors);
    
    m_pursuitFollower = new PurePursuit(k_ticksFootSpark, m_sensors, m_posTracker);

    followThread.start();
  }

  private boolean m_runFollowThread;
	
	public class FollowThread extends Thread{
		public void run() {
			long step = k_mpPeriod;
			for(long nextRun = System.currentTimeMillis();;nextRun += step) {
				if(m_runFollowThread) {
          m_velContainer = m_pursuitFollower.followPath();
          // SmartDashboard.putNumber("Ideal Left Vel", getLeftVel());
          // SmartDashboard.putNumber("Ideal Right Vel", getRightVel());
					setSpeed(m_velContainer.leftVel, m_velContainer.rightVel);
				}
				try {
					long sleepTime = nextRun - System.currentTimeMillis();
					if(sleepTime > 0) {
						sleep(sleepTime);
					}
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

  public void setPower(double leftPower, double rightPower){
    if(m_leftPIDController.isEnabled()){
      disablePID();
    }
    m_leftDrive.set(leftPower);
    m_rightDrive.set(rightPower);
    // System.out.println(String.format("Set Power: %f %f", leftPower, rightPower));
    // Logger.Log("DriveSubsystem", 1, String.format("lp=%f,rp=%f", leftPower, rightPower));
    
    // m_leftPIDSpark.setReference(leftPower, ControlType.kDutyCycle);
    // m_rightPIDSpark.setReference(rightPower, ControlType.kDutyCycle);
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    if(!m_leftPIDController.isEnabled()){
      enablePID();
    }

    setNormalPID();

    m_leftPIDController.setSetpoint(leftSpeed);
    m_rightPIDController.setSetpoint(rightSpeed);
    // System.out.println(String.format("Set Speed: %f %f", leftSpeed, rightSpeed));
    // Logger.Log("DriveSubsystem", 1, String.format("ls=%f,rs=%f", leftSpeed, rightSpeed));

    // if(leftSpeed < 1500 && rightSpeed < 1500){
    //   setSlowPID();
    // }else{
    //   setNormalPID();
    // }
    // setSlowPID();
    // m_leftPIDSpark.setReference(leftSpeed, ControlType.kVelocity);
    // m_rightPIDSpark.setReference(rightSpeed, ControlType.kVelocity);
  }

  public void setSpeedSlow(double leftSpeed, double rightSpeed){
    if(!m_leftPIDController.isEnabled()){
      enablePID();
    }

    setSlowPID();

    m_leftPIDController.setSetpoint(leftSpeed);
    m_rightPIDController.setSetpoint(rightSpeed);
    // System.out.println(String.format("Set Speed: %f %f", leftSpeed, rightSpeed));
    // Logger.Log("DriveSubsystem", 1, String.format("ls=%f,rs=%f", leftSpeed, rightSpeed));

    // if(leftSpeed < 1500 && rightSpeed < 1500){
    //   setSlowPID();
    // }else{
    //   setNormalPID();
    // }
    // setSlowPID();
    // m_leftPIDSpark.setReference(leftSpeed, ControlType.kVelocity);
    // m_rightPIDSpark.setReference(rightSpeed, ControlType.kVelocity);
  }

  public void stop(){
    System.out.println("Stop");
    // new Exception().printStackTrace();
    
    if(m_leftPIDController.isEnabled()){
      disablePID();
    }

    setPower(0, 0);

    // m_leftPIDSpark.setReference(0, ControlType.kDutyCycle);
    // m_rightPIDSpark.setReference(0, ControlType.kDutyCycle);
  }

  public void disablePID(){
    // m_rightPIDController.writerFinish();

    m_leftPIDController.disable();
    m_rightPIDController.disable();
  }

  public void enablePID(){
    // m_rightPIDController.writerStart();

    m_leftPIDController.enable();
    m_rightPIDController.enable();
  }

  public void setSlowPID(){
    if(m_normalPid){
      m_leftPIDSpark.setFF(k_fLeftSlow);
      m_leftPIDSpark.setP(k_pLeftSlow);
      m_leftPIDSpark.setI(k_iLeftSlow);
  
      m_rightPIDSpark.setFF(k_fRightSlow);
      m_rightPIDSpark.setP(k_pRightSlow);
      m_rightPIDSpark.setI(k_iRightSlow);

      m_normalPid = false;
      Logger.Log("DriveSubsystem", 3, "Set Slow Pid");
    }
  }

  public void setNormalPID(){
    if(!m_normalPid){
      m_leftPIDSpark.setFF(k_fLeft);
      m_leftPIDSpark.setP(k_pLeft);
      m_leftPIDSpark.setI(k_iLeft);
  
      m_rightPIDSpark.setFF(k_fRight);
      m_rightPIDSpark.setP(k_pRight);
      m_rightPIDSpark.setI(k_iRight);

      Logger.Log("DriveSubsystem", 3, "Set Fast Pid");

      m_normalPid = true;
    }
  }

  public void setBrakeMode(){
    m_leftDrive.setIdleMode(IdleMode.kBrake);
    m_leftDriveFollower.setIdleMode(IdleMode.kBrake);
    m_rightDrive.setIdleMode(IdleMode.kBrake);
    m_rightDriveFollower.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(){
    m_leftDrive.setIdleMode(IdleMode.kCoast);
    m_leftDriveFollower.setIdleMode(IdleMode.kCoast);
    m_rightDrive.setIdleMode(IdleMode.kCoast);
    m_rightDriveFollower.setIdleMode(IdleMode.kCoast);
  }

  public boolean printPath(Path path) {
		Field[] k_fields = {
				new Field("x", 'f'),
				new Field("Y", 'f')
		};
		CSVWriter writer = new CSVWriter("Path", k_fields);
		writer.start();
		System.out.println(String.format("printPath: length = %d", path.m_centerPath.length));
		for(int i=0; i<path.m_centerPath.length; i++) {
//			System.out.println(path.m_centerPath[i].x+", "+path.m_centerPath[i].y);
			writer.write(path.m_centerPath[i].x, path.m_centerPath[i].y);
		}
		writer.finish();
		return true;
  }

  public double getLeftPos(){
    return -m_leftSparkEncoder.getPosition();
  }

  public double getRightPos(){
    return m_rightSparkEncoder.getPosition();
  }

  public double getLeftVel(){
    return -m_leftSparkEncoder.getVelocity();
  }

  public double getRightVel(){
    return m_rightSparkEncoder.getVelocity();
  }

  public double getLeftPosGrey(){
    return m_leftGreyEncoder.get();
  }

  public double getRightPosGrey(){
    return m_rightGreyEncoder.get();
  }

  public double getAngle() {
		return m_gyro.getFusedHeading();
  }
  
  public double getPitch(){
    double[] data = new double[3];
    m_gyro.getYawPitchRoll(data);
    return -data[1];
  }
	
	public void setGyro(double angle) {
    m_gyro.setFusedHeading(angle * 64.0, RobotMap.k_CTREimeout);
    // System.out.println(String.format("setGyro: set = %f, read = %f", angle, getAngle()));
  }
  
  public double getAngleSPI() {
		return m_gyroSPI.getYaw();
  }

  public void setGyroSPI(double angle){
    m_gyroSPI.resetYaw(angle);
  }
  
  public double getRawAngleSPI() {
    return m_gyroSPI.getYawRaw();
  }

  public void setXY(double x, double y) {
		m_posTracker.setXY(x, y);
  }

  public void cameraUpdate(PositionContainer pos, CameraDirection camera){
    m_posTracker.cameraUpdate(pos, camera);
  }
  
  public void setAngle(double angle) {
    m_posTracker.setAngle(angle);
  }
  
  public void resetPreviousPosition() {
    m_posTracker.resetPreviousPosition();
  }

  public void setCameraCorrection(boolean cameraCorrection){
    m_posTracker.setCameraCorrection(cameraCorrection);
  }
  
  public void startFollow() {
    m_runFollowThread = true;
    m_pursuitFollower.startPath();
  }
  
  public void stopFollow() {
    m_runFollowThread = false;
    m_pursuitFollower.stopFollow();
  }
  
  public void loadFollowPath(Path path, boolean isReversed, boolean isExtended) {
    m_pursuitFollower.loadPath(path, isReversed, isExtended);
  }
  
  public boolean isFollowFinished() {
    return m_pursuitFollower.isFinished();
  }
  
  public void startPosUpdate() {
    m_posTracker.startPosUpdate();
  }

  public void stopPosUpdate() {
    m_posTracker.stopPosUpdate();
  }
  
  public PositionContainer getPos() {
    return m_posTracker.getPos();
  }

  public boolean getReversed(){
    return m_reversed;
  }

  public void setReversed(boolean reversed){
     m_reversed = reversed;
  }

  public double ticksToFeetSpark(double ticks) {
		return ticks / k_ticksFootSpark;
  }
	
	public double feetToTicksSpark(double feet) {
		return feet * k_ticksFootSpark;
  }
  
  public double ticksToFeetGrey(double ticks) {
		return ticks / k_ticksFootGrey;
  }
  
  public double getAccelY(){
    return m_accel.getY();
  }
}
