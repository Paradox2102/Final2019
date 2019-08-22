package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;

public class Sensor implements SensorData{
	private CANEncoder m_leftSparkEncoder;
	private CANEncoder m_rightSparkEncoder;
	private Encoder m_leftGreyEncoder;
	private Encoder m_rightGreyEncoder;
	private PigeonIMU m_gyro;
	
	public Sensor(CANEncoder leftSparkEncoder, CANEncoder rightSparkEncoder, Encoder leftGreyEncoder, Encoder rightGreyEncoder, PigeonIMU gyro) {
		m_leftSparkEncoder = leftSparkEncoder;
		m_rightSparkEncoder = rightSparkEncoder;
		
		m_leftGreyEncoder = leftGreyEncoder;
		m_rightGreyEncoder = rightGreyEncoder;
		
		m_gyro = gyro;
	}
	
	public double getLeftEncoderPos() {
		return m_leftGreyEncoder.get();
	}
	
	public double getRightEncoderPos() {
		return m_rightGreyEncoder.get();
	}
	
	public double getLeftEncoderVel() {
		return -m_leftSparkEncoder.getVelocity()/60;
	}
	
	public double getRightEncoderVel() {
		return m_rightSparkEncoder.getVelocity()/60;
	}
	
	public double getAngle() {
		return m_gyro.getFusedHeading();
	}
}
