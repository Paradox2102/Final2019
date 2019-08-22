package frc.robot;

import frc.lib.CSVWriter;
import frc.lib.CSVWriter.Field;
import frc.robot.Navigator.CameraDirection;
import frc.robot.Navigator.Target;

public class PositionTracker implements Tracker{
	private double m_x;
	private double m_y;
	private double m_xraw;
	private double m_yraw;
	
	private double k_ticksFoot;
	
	private double m_prevLeft;
	private double m_prevRight;
	
	private double m_lastAngle;
	
	private double k_encoderWeight = 9;
	private double k_cameraWeight = 1;

	private boolean m_cameraCorrection = false;
	
	private boolean m_runPositionThread;
	
	private Object lock = new Object();
	
	private PositionThread m_thread = new PositionThread();
	
	private SensorData m_sensors;

	private Field[] k_fieldsPos = {
		new Field("Function", 's'),
		new Field("cur left", 'f'),
		new Field("prev left", 'f'),
		new Field("cur right", 'f'),
		new Field("prev right", 'f'),
		new Field("m_x", 'f'),
		new Field("m_y", 'f')
	};

	private Field[] k_fieldsCamera = {
		new Field("Camera X", 'f'),
		new Field("Encoder X", 'f'),
		new Field("Raw x", 'f'),
		new Field("Camera Y", 'f'),
		new Field("Encoder Y", 'f'),
		new Field("Raw y", 'f'),
		new Field("Pixels Apart", 'f'),
		new Field("offset", 'f'),
		new Field("gyro", 'f'),
		new Field("dx", 'f'),
		new Field("dy", 'f')
	};

	private CSVWriter posWriter = new CSVWriter("Position Data", k_fieldsPos);
	private CSVWriter cameraWriter = new CSVWriter("Pos and Camera Data", k_fieldsCamera);
	
	public PositionTracker(double x, double y, double ticksFeet, boolean cameraCorrection, SensorData sensor) {
		m_sensors = sensor;
		
		m_x = m_xraw =x;
		m_y = m_yraw = y;
		
		k_ticksFoot = ticksFeet;
		
		m_prevLeft = getLeftEncoderPos();
		m_prevRight = getRightEncoderPos();
		m_lastAngle = getAngle();

		m_cameraCorrection = cameraCorrection;
		
		m_thread.start();
	}
	
	private void updatePos() {
		double leftPos = getLeftEncoderPos();
		double rightPos = getRightEncoderPos();
		// System.out.println("Left Prev " + m_prevLeft + "\nRight Prev " + m_prevRight);
		// System.out.println("Left Pos " + leftPos + "\nRight " + rightPos);
		synchronized(lock) {

			double dist = ((leftPos - m_prevLeft) + (rightPos - m_prevRight))/2.0;
			
			double curAngle = (getAngle() + m_lastAngle)/2.0;
			
			curAngle = Math.toRadians(curAngle);

			// if (Math.abs(dist) > 0.001)
			// {
			// 	System.out.println(String.format("distxxx=%f", dist));
			// }
		
			m_x += (dist * Math.cos(curAngle));
			m_y += (dist * Math.sin(curAngle));
			m_xraw += (dist * Math.cos(curAngle));
			m_yraw += (dist * Math.sin(curAngle));

			// posWriter.write("Update", ticksToFeet(leftPos), ticksToFeet(m_prevLeft), ticksToFeet(rightPos), ticksToFeet(m_prevRight), ticksToFeet(m_x), ticksToFeet(m_y));
			
	//		System.out.println(String.format("Dist: %f, x: %f, y%f", ticksToFeet(dist), ticksToFeet(m_x), ticksToFeet(m_y)));
			
			m_prevLeft = leftPos;
			m_prevRight = rightPos;
			m_lastAngle = getAngle();
			
	//		PositionWriter.write(ticksToFeet(leftPos), ticksToFeet(rightPos), ticksToFeet(dist), Math.toDegrees(curAngle), ticksToFeet(m_x), ticksToFeet(m_y));
		}
	}
	
	public PositionContainer getPos() {
		synchronized (lock) {
			return new PositionContainer(ticksToFeet(m_x), ticksToFeet(m_y));
		}
	}
	
	public void setXY(double x, double y) {
		synchronized (lock) {
			m_x = feetToTicks(x);
			m_y = feetToTicks(y);
			m_xraw = feetToTicks(x);
			m_yraw = feetToTicks(y);
			// posWriter.write("Set", ticksToFeet(getLeftEncoderPos()),ticksToFeet(m_prevLeft), ticksToFeet(getRightEncoderPos()), ticksToFeet(m_prevRight), ticksToFeet(m_x), ticksToFeet(m_y));
		}
	}

	public void cameraUpdate(PositionContainer poss, CameraDirection direction) {
		synchronized (lock) {
			if(m_cameraCorrection){
				PositionContainer pos = Robot.m_navigator.getCamPos();
				pos.x = feetToTicks(pos.x);
				pos.y = feetToTicks(pos.y);
				Target target = Robot.m_navigator.getCorrectTarget(direction);
				double dx = Math.abs(pos.x - m_x);
				double dy = Math.abs(pos.y - m_y);
				// cameraWriter.write(ticksToFeet(pos.x), ticksToFeet(m_x), ticksToFeet(m_xraw), ticksToFeet(pos.y), ticksToFeet(m_y), ticksToFeet(m_yraw), Robot.m_navigator.getTargetPixelsApartCalibration(direction), Robot.m_navigator.getTargetCenterOffset(target.m_pair, direction), Robot.m_driveSubsystem.getAngle(), dx, dy);

				if ((dx > feetToTicks(0.5)) || (dy > feetToTicks(0.5)))
				{
					m_x = (m_x * k_encoderWeight + pos.x * k_cameraWeight)/(k_encoderWeight + k_cameraWeight);
					m_y = (m_y * k_encoderWeight + pos.y * k_cameraWeight)/(k_encoderWeight + k_cameraWeight);
				}
				// System.out.println(String.format("Camera x: %f, Encoder x: %f, Camera y: %f, Encoder y: %f", pos.x, m_x, pos.y, m_y));
			}
		}
	}

	public void setCameraCorrection(boolean cameraCorrection){
		synchronized (lock) {
			m_cameraCorrection = cameraCorrection;
		}
	}
	
	public void setAngle(double angle) {
		synchronized (lock) {
			m_lastAngle = angle;
		}
	}
	
	public void resetPreviousPosition() {
		synchronized (lock) {
			m_prevLeft = getLeftEncoderPos();
			m_prevRight = getRightEncoderPos();	
		}
	}
	
	private double getLeftEncoderPos() {
		return m_sensors.getLeftEncoderPos();
	}
	
	private double getRightEncoderPos() {
		return m_sensors.getRightEncoderPos();
	}
	
	private double getAngle() {
		return m_sensors.getAngle();
	}
	
	private double ticksToFeet(double ticks) {
		return ticks / k_ticksFoot;
	}
	
	private double feetToTicks(double feet) {
		return feet * k_ticksFoot;
	}
	
	public static class PositionContainer{
		public double x, y;
		
		public PositionContainer(double x, double y) {
			this.x = x;
			this.y = y;
		}
	}
	
	public class PositionThread extends Thread{
		public void run() {
			long step = 10;
			for(long nextRun = System.currentTimeMillis();;nextRun += step) {
				if(m_runPositionThread) {
					updatePos();
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
	
	public void startPosUpdate() {
		// posWriter.start();
		// cameraWriter.start();

		// posWriter.write("Start", ticksToFeet(getLeftEncoderPos()),ticksToFeet(m_prevLeft), ticksToFeet(getRightEncoderPos()), ticksToFeet(m_prevRight), ticksToFeet(m_x), ticksToFeet(m_y));
		m_runPositionThread = true;
	}
	
	public void stopPosUpdate() {
		// posWriter.finish();
		// cameraWriter.finish();

		m_runPositionThread = false;
	}
	
}