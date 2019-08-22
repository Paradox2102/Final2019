package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.PIDControllerIntake;
import frc.robot.RobotMap;
import frc.robot.SparkMaxWrapperPos;
import frc.robot.commands.intake.StallIntakeCommand;

public class IntakeSubsystem extends Subsystem {
  private static final CANSparkMax m_intakeArm = new CANSparkMax(RobotMap.k_intakeArm, MotorType.kBrushless);
  private static CANDigitalInput m_intakeLimit;
  private static final Encoder m_encoder = new Encoder(RobotMap.k_intakeEncoderA, RobotMap.k_intakeEncoderB);

  private PIDControllerIntake m_pidController;
  
  SparkMaxWrapperPos m_sparkWrapper;

  CANEncoder m_intakeSparkEncoder;

  private int m_home;

  private final double k_maxPower = 1;
  private final double k_minPower = -1;
  private final int k_minRange = (int)degToTicks(220);
  private final int k_maxRange = (int) degToTicks(-20);
  private final double k_error = deltaDegToTicks(2);
  private final double k_p = 1.0/1000.0;
  private final double k_iDown = 0;
  private final double k_iRange = 0;//degToTicks(10);
  private final double k_d = 0;
  private final double k_f = 0.015;

  private double m_lastIntakePos;
  private boolean m_stall = false;

  public IntakeSubsystem(){
    m_intakeArm.setInverted(false);
    m_intakeArm.setIdleMode(IdleMode.kBrake);

    m_intakeArm.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    m_intakeLimit = m_intakeArm.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    m_encoder.setReverseDirection(true);

    m_intakeArm.setIdleMode(IdleMode.kBrake);

    m_home = m_encoder.get();

    m_intakeSparkEncoder = m_intakeArm.getEncoder();
    m_sparkWrapper = new SparkMaxWrapperPos(m_intakeArm, m_encoder);

    m_pidController = new PIDControllerIntake(k_p, k_iDown, k_d, k_f, m_sparkWrapper, m_sparkWrapper);

    m_pidController.setIRange(k_iRange);

    m_pidController.setInputRange(k_minRange, k_maxRange);
    m_pidController.setOutputRange(k_minPower, k_maxPower);
    m_pidController.setAbsoluteTolerance(k_error);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new StallIntakeCommand(RobotMap.k_finalRobot));
  }

  public void set(double power){
    if(m_pidController.isEnabled()){
      disablePid();
    }
    m_intakeArm.set(power);
    m_stall = false;
  }

  public void setPos(double deg){
    double ticks = degToTicks(deg);
    if(!m_pidController.isEnabled()){
      enablePid();
    }

    m_lastIntakePos = deg;

    m_pidController.setSetpoint(ticks);
  }

  public void setPosFinal(double deg){
    double ticks = degToTicksFinal(deg);
    if(!m_pidController.isEnabled()){
      enablePid();
    }

    // if(deg == 90){
    //   m_pidController.setI(k_iUp);
    // }else{
    //   m_pidController.setI(k_iDown);
    // }
    m_lastIntakePos = deg;

    m_pidController.setSetpoint(ticks);
    // SmartDashboard.putNumber("Ideal Pos", ticks);
    // m_arm.set(ControlMode.Position, degToTicks(deg));
  }
  
  public void stop(){
    disablePid();
    m_intakeArm.set(0);
  }

  public boolean stall(){
    return m_stall;
  }

  public void setStall(boolean stall){
    m_stall = stall;
  }

  public double lastPos(){
    return m_lastIntakePos;
  }

  public void brakeMode(){
    m_intakeArm.setIdleMode(IdleMode.kBrake);
  }

  public void coastMode(){
    m_intakeArm.setIdleMode(IdleMode.kCoast);
  }

  public boolean getLimit(){
    boolean limit = m_intakeLimit.get();
    if(limit){
      m_stall = false;
    }
    return limit; 
  }

  public void homeEncoder(){
    m_home = m_encoder.get();
    m_sparkWrapper.setOffset(m_home);
  }

  public int getPosRelativeHome(){
    return m_encoder.get() - m_home;
  }

  public void enablePid(){
    m_pidController.enable();
  }

  public void disablePid(){
    m_pidController.disable();
  }

  public static double ticksToDeg(double pos){
    return -0.06315 * pos + 203.68;
  }

  public static double degToTicks(double deg){
    return -15.833 * deg + 3225;
  }

  public static double ticksToDegFinal(double pos){
    return -0.0653 * pos + 204.165;
  }

  public static double degToTicksFinal(double deg){
    return -15.311 * deg + 3126;
  }

  public static double deltaDegToTicks(double deg){
    return 15.833 * deg;
  }
  
  public double getAngle(){
    return ticksToDeg(getPosRelativeHome());
  }

  public double getAngleFinal(){
    return ticksToDegFinal(getPosRelativeHome());
  }

  public boolean intakeVertical(){
    return Math.abs(getAngle() - RobotMap.k_intakeUpPos) <= 10;
  }
}
