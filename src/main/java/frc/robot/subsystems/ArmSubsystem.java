/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.lib.CSVWriter;
import frc.lib.PIDControllerArm;
import frc.lib.PIDControllerArmFinal;
import frc.lib.CSVWriter.Field;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.arm.StallArmCommand;

/**
 * Add your docs here.
 */
public class ArmSubsystem extends Subsystem implements PIDSource, PIDOutput {
  private static final TalonSRX m_arm = new TalonSRX(RobotMap.k_arm);
  
  private PIDControllerArm m_pidController;
  private PIDControllerArmFinal m_pidControllerFinal;

  private final static DigitalInput m_hatchLimitSwitch = new DigitalInput(RobotMap.k_hatchLimitSwitch);

  private int m_offset = 0;
  private double m_lastPos;
  private double m_lastPower = 0;
  private long m_lastCall;

  private final double k_maxPower = 1;
  private final double k_minPower = -1;
  private final int k_minRange = 15;
  private final int k_maxRange = 960;
  private final double k_error = deltaDegToTicks(1);
  private final double k_p = 0.00185 * 1.1;
  private final double k_pUp = 0.00185 * 1.25;
  private final double k_iUp = 0.0001 * 1;
  private final double k_iDown = 0;
  private final double k_iRange = deltaDegToTicks(20);
  private final double k_iRangeFinal = deltaDegToTicks(20);
  private final double k_d = 0;//0.01;
  private final double k_fLoaded = 0.12 * 1.25;//.2 //0.55;    Unloaded: 0.08
  private final double k_fUnloaded = 0.0655;//.2 //0.55;
  //switch between constants for final and this robot for f terms
  // private final double k_p = 0.00185 * 1.5;
  // private final double k_iUp = 0.0001;
  // private final double k_iDown = 0.0004;
  // private final double k_iRange = deltaDegToTicks(20);
  // private final double k_iRangeFinal = deltaDegToTicks(20);
  // private final double k_d = 0;//0.01;
  // private final double k_f = 0.15;//.2 //0.55;
  private final int k_pidSlot = 0;

  private final int k_posCheck = 400;
  
  private double m_lastArmPos = 90;
  private boolean m_stall = false;

  private boolean m_disabled = false;
  private boolean m_checkDisabled = true;

  private Field[] k_fields = {
    new Field("Arm Pos", 'd'),
    new Field("Offset", 'd')
  };
  private CSVWriter writer = new CSVWriter("Arm Offset", k_fields);

  public ArmSubsystem(){
    m_arm.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, RobotMap.k_CTREimeout);
    m_arm.configFeedbackNotContinuous(true, RobotMap.k_CTREimeout);
    m_arm.setSensorPhase(true);
    m_arm.setInverted(true);
    m_arm.setNeutralMode(NeutralMode.Brake);

    m_arm.configNominalOutputForward(0);
    m_arm.configNominalOutputReverse(0);

    m_arm.configPeakOutputForward(k_maxPower);
    m_arm.configPeakOutputReverse(k_minPower);

    m_arm.config_kP(k_pidSlot, k_p);
    m_arm.config_kI(k_pidSlot, k_iDown);
    m_arm.config_kD(k_pidSlot, k_d);
    m_arm.config_kF(k_pidSlot, k_fLoaded);

    // m_arm.configAllowableClosedloopError((int)(degToTicks(5)), k_pidSlot, RobotMap.k_CTREimeout);

    m_arm.selectProfileSlot(k_pidSlot, 0);

    m_pidController = new PIDControllerArm(k_p, k_iDown, k_d, k_fLoaded, this, this);
    m_pidControllerFinal = new PIDControllerArmFinal(k_p, k_iDown, k_d, k_fLoaded, this, this);

    m_pidController.setIRange(k_iRange);

    m_pidController.setInputRange(k_minRange, k_maxRange);
    m_pidController.setOutputRange(k_minPower, k_maxPower);
    m_pidController.setAbsoluteTolerance(k_error);

    m_pidControllerFinal.setIRange(k_iRangeFinal);
    m_pidControllerFinal.setInputRange(k_minRange, k_maxRange);
    m_pidControllerFinal.setOutputRange(k_minPower, k_maxPower);
    m_pidControllerFinal.setAbsoluteTolerance(k_error);

    m_lastPos = getPos();
    m_lastCall = System.currentTimeMillis();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new StallArmCommand(RobotMap.k_finalRobot));
  }

  public void setPower(double power, boolean pidControl){
    if(!m_disabled){
      m_lastPower = power;

      if(!pidControl){
        if(m_pidController.isEnabled()){
          disablePid();
        }
  
        if(m_pidControllerFinal.isEnabled()){
          disablePidFinal();
        }
      }
  
      // System.out.println(String.format("p=%f,o=%d"  power, m_offset));
  
      if(power > 0 && m_offset > 0){
        power = 0;
      } else if(power < 0 && m_offset < 0){
        power = 0;
      }
  
      m_stall = false;
  
      m_arm.set(ControlMode.PercentOutput, power);
    }

  }

  public void setPos(double deg){
    if(!m_disabled){
      double ticks = degToTicks(deg);
      if(!m_pidController.isEnabled()){
        enablePid();
      }
  
      if(hasHatch()){
        m_pidController.setF(k_fLoaded);
      }else{
        m_pidController.setF(k_fUnloaded);
      }
  
      if(deg == 90){
        m_pidController.setI(k_iUp);
        m_pidController.setP(k_pUp);
      }else{
        m_pidController.setI(k_iDown);
        m_pidController.setP(k_p);
      }
  
      m_lastArmPos = deg;
  
      m_pidController.setSetpoint(ticks);
      // SmartDashboard.putNumber("Ideal Pos", ticks);
      // m_arm.set(ControlMode.Position, degToTicks(deg));
    }
  }

  public void powerOverride(double power){
    if(m_pidController.isEnabled()){
      disablePid();
    }

    if(m_pidControllerFinal.isEnabled()){
      disablePidFinal();
    }
    
    m_arm.set(ControlMode.PercentOutput, power);
  }

  public void setPosFinal(double deg){
    if(!m_disabled){
      double ticks = degToTicksFinal(deg);
    if(!m_pidControllerFinal.isEnabled()){
      enablePidFinal();
    }

    if(hasHatch()){
      m_pidControllerFinal.setF(k_fLoaded);
    }else{
      m_pidControllerFinal.setF(k_fUnloaded);
    }

    if(deg == 90){
      m_pidControllerFinal.setI(k_iUp);
      m_pidControllerFinal.setP(k_pUp);
    }else{
      m_pidControllerFinal.setI(k_iDown);
      m_pidControllerFinal.setP(k_p);
    }

    m_lastArmPos = deg;

    m_pidControllerFinal.setSetpoint(ticks);
    // SmartDashboard.putNumber("Ideal Pos", ticks);
    // m_arm.set(ControlMode.Position, degToTicks(deg));
    }
  }

  public void enablePid(){
    m_pidController.enable();
  }

  public void enablePidFinal(){
    m_pidControllerFinal.enable();
  }

  public void disablePid(){
    m_pidController.disable();
  }

  public void disablePidFinal(){
    m_pidControllerFinal.disable();
  }

  public void stop(){
    disablePid();
    disablePidFinal();

    m_lastPower = 0;
    m_arm.set(ControlMode.PercentOutput, 0);
  }

  public boolean stall(){
    return m_stall;
  }

  public void setStall(boolean stall){
    m_stall = stall;
  }

  public double lastPos(){
    return m_lastArmPos;
  }

  public void setLastPos(){
    if(RobotMap.k_finalRobot){
      m_lastArmPos = getAngleFinal();
    }else{
      m_lastArmPos = getAngle();
    }
  }

  public int getPos(){
    return m_arm.getSelectedSensorPosition();
  }

  public int getPosRelative(){
    return m_arm.getSelectedSensorPosition() + m_offset;
  }

  public void disableMotor(){
    if(m_checkDisabled){
      m_disabled = true;
      stop();
    }
  }

  public void enableMotor(){
    m_disabled = false;
    stop();
  }

  public void checkDisable(boolean check){
    m_checkDisabled = check;
  }

  public static double ticksToDeg(double pos){
    return -0.336*pos + 219.956;
  }

  public double degToTicks(double deg){
    return -2.972*deg + 653.833;
  }

  public static double ticksToDegFinal(double pos){
    return -0.336*pos + 215.26;
  }

  public double degToTicksFinal(double deg){
    return -2.978*deg + 641;
  }

  public double deltaDegToTicks(double deg){
    return 2.978 * deg;
  }

  public double getAngle(){
    return ticksToDeg(getPosRelative());
  }

  public double getAngleFinal(){
    return ticksToDegFinal(getPosRelative());
  }

  public double getOutput(){
    return m_arm.getMotorOutputVoltage();
  }

  public boolean isOnTarget(){
    return m_pidController.onTarget();
  }

  public void checkLimit(){
    long curTime = System.currentTimeMillis();
    if((curTime - m_lastCall) > 100){
      int curPos = getPos();
      
      if(m_lastPos - curPos > k_posCheck){
        m_offset += 1024;
      }else if(m_lastPos - curPos < -k_posCheck){
        m_offset -= 1024;
      }

      if(m_lastPower > 0 && m_offset > 0){
        stop();
      } else if(m_lastPower < 0 && m_offset < 0){
        stop();
      }

      m_lastPos = curPos;

      // SmartDashboard.putNumber("Offset", m_offset);
      // writer.write(curPos, m_offset);

      m_lastCall = curTime;
    }
  }

  public boolean hasHatch(){
    return m_hatchLimitSwitch.get();
  }

  public void stopWriter(){
    // writer.finish();
  }

  public void write(){
    // writer.start();
  }

  @Override
  public void pidWrite(double output) {
    setPower(output, true);
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {

  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return PIDSourceType.kDisplacement;
  }

  @Override
  public double pidGet() {
    return getPosRelative();
}
}
