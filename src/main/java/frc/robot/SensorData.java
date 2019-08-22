package frc	.robot;

public interface SensorData {
	double getLeftEncoderPos();
	double getRightEncoderPos();
	double getLeftEncoderVel();
	double getRightEncoderVel();
	double getAngle();
}
