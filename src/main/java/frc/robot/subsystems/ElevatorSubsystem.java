package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.PIDControllerElevator;
import frc.robot.RobotMap;
import frc.robot.SparkMaxWrapperPos;
import frc.robot.commands.elevator.TeleopElevatorCommand;

/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends Subsystem implements PIDSource, PIDOutput{
  private static final CANSparkMax m_elevator = new CANSparkMax(RobotMap.k_elevator, MotorType.kBrushless);
  private static final Encoder m_encoder = new Encoder(RobotMap.k_elevatorEncoderA, RobotMap.k_elevatorEncoderB);
  private static DigitalInput m_revLimit;
  private PIDControllerElevator m_pidController;
  private SparkMaxWrapperPos m_sparkWrapper;
  private Solenoid m_brake = new Solenoid(RobotMap.k_brake);

  private double k_fUp = 0.1;//0.1;
  private double k_fUpMin = 0.1 * 5;
  private double k_pUp = 0.5/1000 * 0.8;//1;//0.8;
  private double k_pUpMin = 0.5/1000 * 4;
  private double k_iUpMin = 0.1 / 1000 * 1.5;
  private double k_iUp = 0.1 / 1000 * 0;//1;//25;
  private double k_dUp = 0;

  private double k_fDown = 0.1;
  private double k_pDown = 0.25 / 1000 * 0.8;//1.2;
  private double k_iDown = 0.075 / 1000 * 1;
  private double k_dDown = 0;

  private double k_iRange = 400;
  private double k_maxPower = 1;
  private final double k_minPower = -0.5;
  private final int k_minRange = 0;
  private final int k_maxRange = 15700;
  private final double k_error = 100;
  private final int k_minPid = 500;

  private int m_elevatorHome;

  public ElevatorSubsystem(){
    m_elevator.setInverted(true);
    m_elevator.setIdleMode(IdleMode.kBrake);

    m_elevator.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
    m_revLimit = new DigitalInput(RobotMap.k_elevatorLimit);

    m_sparkWrapper = new SparkMaxWrapperPos(m_elevator, m_encoder);

    m_pidController = new PIDControllerElevator(k_pUp, k_iUp, k_dUp, k_fUp, m_sparkWrapper, m_sparkWrapper);

    m_pidController.setIRange(k_iRange);
    m_pidController.setInputRange(k_minRange, k_maxRange);
    m_pidController.setOutputRange(k_minPower, k_maxPower);
    m_pidController.setAbsoluteTolerance(k_error);

    m_encoder.setReverseDirection(true);

    // m_elevator.setSmartCurrentLimit(65);

    m_elevatorHome = m_encoder.get();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopElevatorCommand());
  }

  public void set(double power){
    if(m_pidController.isEnabled()){
      disablePid();
    }
    releaseBrake();

    if(!getRevLimit() && power<0){
      power = 0;
    }

    m_elevator.set(power);
  }

  public void setPos(double pos){
    if(pos > getPosRelativeHome()){
      if(pos - getPosRelativeHome() > k_minPid){
        setPidUp();
      }else{
        setPidUpMinimal();
      }
    }else{
      setPidDown();
    }

    if(!m_pidController.isEnabled()){
      enablePid();
    }
    releaseBrake();
    m_pidController.setSetpoint(pos);
  }

  public void enablePid(){
    m_pidController.enable();
  }

  public void disablePid(){
    m_pidController.disable();
  }

  public void setPidUpMinimal(){
    setMaxOutput(1);
    m_pidController.setPID(k_pUpMin, k_iUpMin, k_dUp, k_fUpMin);
  }

  public void setPidUp(){
    setMaxOutput(1);
    m_pidController.setPID(k_pUp, k_iUp, k_dUp, k_fUp);
  }

  public void setPidDown(){
    m_pidController.setPID(k_pDown, k_iDown, k_dDown, k_fDown);
  }

  //TODO: Create Elevator PID
  public void setSpeed(double speed){
    releaseBrake();
  }

  public void stop(){
    m_elevator.set(0);
    disablePid();
    brake();
  }

  public void brake(){
    m_brake.set(false);
  }

  public void releaseBrake(){
    m_brake.set(true);
  }

  public boolean getRevLimit(){
    return m_revLimit.get();
  }

  public void homeEncoder(){
    m_elevatorHome = m_encoder.get();
    m_sparkWrapper.setOffset(m_elevatorHome);
  }

  public int getHome(){
    return m_elevatorHome;
  }

  public int getPosRelativeHome(){
    return m_encoder.get() - m_elevatorHome;
  }

  public void setMaxOutput(double power){
    k_maxPower = power;
  }

  @Override
  public void pidWrite(double output) {
    set(output);
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
    return getPosRelativeHome();
  }

  public double getAmp(){
    return m_elevator.getOutputCurrent();
  }
}
