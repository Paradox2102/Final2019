package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.pathfinder.Pathfinder.Path;
import frc.pathfinder.Pathfinder.Segment;
import frc.lib.CSVWriter;
import frc.lib.CSVWriter.Field;
import frc.robot.PositionTracker.PositionContainer;

public class PurePursuit {
	private Path m_path;
	private boolean m_isReversed;
	private boolean m_isExtended;

	private final double k_lookAheadTime = 0.7;//0.3;//.4

	private final double k_lookAheadDist = 0.75;
	private final int k_maxTimeLookAhead = 1;
	
	private final double k_maxVelFast = 8;//13.04
	private final double k_maxVelSlow = 3;//11
	private final double k_maxAcc = 9;//5;  //9;//15;//6;//8.46
	private final double k_maxDeccel = 5;//3;
	private final double k_maxJerk = 50;

	private final double k_width = 26.75/12;
	private final int k_points = 1000;
	private final double k_dt = 0.02;
	
	private final int k_minVel = 1;
	private final int k_checkStall = 40;
	
	private final double k_ticksFoot;
	
	private final int k_lookAheadPoints = (int) (k_maxTimeLookAhead / k_dt);
	private final double k_extendedLookAhead = 0.75;

	private int m_prevIdx = 0;
	private boolean finished = false;
	
	public final PathConfig m_pathConfigFastest = new PathConfig(k_points, 12, 11, 8, k_maxJerk, k_dt, k_width);
	public final PathConfig m_pathConfigFast = new PathConfig(k_points, k_maxVelFast, k_maxAcc, k_maxDeccel, k_maxJerk, k_dt, k_width);
	public final PathConfig m_pathConfigSlow = new PathConfig(k_points, k_maxVelSlow, k_maxAcc, k_maxDeccel, k_maxJerk, k_dt, k_width);
	
	private Field[] k_fields = {
			new Field("Vel", 'f'),
			new Field("Ideal Vel L", 'f'),
			new Field("Ideal Vel R", 'f'),
			new Field("Vel L", 'f'),
			new Field("Vel R", 'f'),
			new Field("Cur X", 'f'),
			new Field("Cur Y", 'f'),
			new Field("Ideal x", 'f'),
			new Field("Ideal y", 'f'),
			new Field("Next Pos x", 'f'),
			new Field("Next Pos y", 'f'),
			new Field("L", 'f'),
			new Field("dX", 'f'),
			new Field("Theta", 'f'),
			new Field("Curvature", 'f'),
			new Field("velDif", 'f'),
			new Field("cur Angle", 'f')
	};
	
	private Tracker m_tracker;
	
	private CSVWriter writer = new CSVWriter("Follow Profile", k_fields);
	
	private SensorData m_sensor;
	
	private PositionContainer m_pos;
	
	public PurePursuit(double ticksFoot, SensorData sensor, Tracker tracker) {
		m_sensor = sensor;
		
		m_tracker = tracker;
		
		k_ticksFoot = ticksFoot;
	}
	
	public void loadPath(Path path, boolean isReversed, boolean isExtended) {
		m_isReversed = isReversed;
		m_path = path;
		m_isExtended = isExtended;
	}

	private int loopCount = 0;
	
	public void startPath() {
		synchronized(lock)
		{
			finished = false;
		}
		loopCount = 0;
		writer.start();
	}
	
	public boolean isFinished() {
		synchronized(lock)
		{
			return finished;
		}
	}
	
	Object lock = new Object();
	
	public void stopFollow() {
		synchronized (lock)
		{
			finished = true;
			writer.finish();
			m_prevIdx = 0;
		}
	}

	private VelocityContainer getEndingVel(){
		double leftVel = m_path.m_leftPath[m_path.m_leftPath.length - 1].velocity;
		double rightVel = m_path.m_rightPath[m_path.m_rightPath.length - 1].velocity;
		System.out.println(String.format("Left Vel: %f Right Vel: %f", feetToTicks(leftVel), feetToTicks(rightVel)));

		if(m_isReversed){
			leftVel *= -1;
			rightVel *= -1;
		}

		return new VelocityContainer(feetToTicks(leftVel*60), feetToTicks(rightVel*60));
	}
	
	public VelocityContainer followPath() {
		synchronized(lock)
		{
			getPos();
			
			if (finished)
			{
				return getEndingVel();
			}
			
			int closestPathIdx = getClosestPoint();
			if(closestPathIdx == -1) {
				DriverStation.reportError("Lost the path!", false);
				stopFollow();
				return new VelocityContainer(0,0);
			}
			Segment closestPos = m_path.m_centerPath[closestPathIdx];
		
			m_prevIdx = closestPathIdx;
			
			if(closestPathIdx == m_path.m_centerPath.length - 1) {
				stopFollow();
				return getEndingVel();
			}
		
			int lookAheadIdx =  closestPathIdx + ((int) (k_lookAheadTime / closestPos.dt));
			
			if(lookAheadIdx > m_path.m_centerPath.length - 1) {
				lookAheadIdx = m_path.m_centerPath.length - 1;
			}
			
			if(closestPathIdx > m_path.m_centerPath.length - k_checkStall && !m_isReversed) {//was 50
				if(Math.abs(ticksToFeet(getRightEncoderVel())) <= 0.1 
						&& Math.abs(ticksToFeet(getLeftEncoderVel())) <= 0.1) {
					stopFollow();
					DriverStation.reportError("Stalled", false);
					return new VelocityContainer(0,0);
				}
			}
			
			Segment nextPos = m_path.m_centerPath[lookAheadIdx];
			double velocity = closestPos.velocity;
			
			if(velocity < k_minVel) {
				velocity = 1;
			}
			
			if(m_isReversed) {
				velocity = -velocity;
			}
			
			double x = m_pos.x;
			double y = m_pos.y;

			double distance = calcDistance(x, y, nextPos);
			if(distance < k_lookAheadDist) {
				int nextPosIdx = getLookAheadPoint(k_lookAheadDist);
				if(nextPosIdx < 0 && m_isExtended) {
					// double curAngle = Math.toRadians(getAngle());
					Segment newPos = m_path.m_centerPath[m_path.m_centerPath.length-1];
					double newX = newPos.x + k_extendedLookAhead * Math.cos(m_path.m_centerPath[m_path.m_centerPath.length-1].heading);
					double newY = newPos.y + k_extendedLookAhead * Math.sin(m_path.m_centerPath[m_path.m_centerPath.length-1].heading);
					nextPos = new Segment(0, newX, newY, 0, 0, 0, 0);
				}else if(nextPosIdx < 0) {
					nextPos = m_path.m_centerPath[m_path.m_centerPath.length-1];
				}else {
					nextPos = m_path.m_centerPath[nextPosIdx];
				}
				distance = calcDistance(x, y, nextPos);
			}

			double angle = getAngle();
			if (loopCount++ == 0)
			{
				System.out.println(String.format("first angle = %f", angle));
			}
			double theta = Math.atan2(nextPos.y - y, nextPos.x - x);
			double dX = distance * Math.sin(theta - Math.toRadians(!m_isReversed ? angle : angle + 180));
			double curvature = (2 * dX) / (distance * distance); 
			double velDif = velocity * curvature * (k_width / 2.0);
			double leftVel = !m_isReversed ? velocity - velDif : velocity + velDif;
			double rightVel = !m_isReversed ? velocity + velDif : velocity - velDif;
						
			writer.write(velocity, (leftVel), (rightVel), -1 * ticksToFeet(getLeftEncoderVel()), 
					ticksToFeet(getRightEncoderVel()), m_pos.x, m_pos.y, closestPos.x, closestPos.y, nextPos.x, nextPos.y,
					distance, dX, theta, curvature, velDif, angle);
			return new VelocityContainer(feetToTicks(leftVel*60), feetToTicks(rightVel*60));
		}
	}
	
	private int getClosestPoint() {
		double smallestDistance = Double.MAX_VALUE;
		int closestIdx = -1;
		double x = m_pos.x;
		double y = m_pos.y;

		int lookAheadPoints = m_prevIdx + k_lookAheadPoints > m_path.m_centerPath.length ? m_path.m_centerPath.length :  m_prevIdx + k_lookAheadPoints;
		
		for(int i=m_prevIdx; i<lookAheadPoints; i++) {
			double distance = calcDistance(x, y, m_path.m_centerPath[i]);
			if(distance < smallestDistance) {
				smallestDistance = distance;
				closestIdx = i;
			}
		}
		
		return closestIdx;
	}
	
	private int getLookAheadPoint(double lookAheadFt) {
		double x = m_pos.x;
		double y = m_pos.y;

		int closestPoint = getClosestPoint();
		int lookAheadPoints = closestPoint + k_lookAheadPoints > m_path.m_centerPath.length ? m_path.m_centerPath.length :  closestPoint + k_lookAheadPoints;
		
		for(int i=closestPoint; i<lookAheadPoints; i++) {
			if(calcDistance(x, y, m_path.m_centerPath[i]) >= lookAheadFt) {
				return i;
			}
		}
		return -1;
		
	}
	
	private double calcDistance(double x, double y, Segment seg) {
		double distance = Math.sqrt(Math.pow((seg.y - y), 2) + Math.pow((seg.x - x), 2));
		return distance;
	}
	
	private double getLeftEncoderVel() {
		return m_sensor.getLeftEncoderVel();
	}
	
	private double getRightEncoderVel() {
		return m_sensor.getRightEncoderVel();
	}
	
	private double getAngle() {
		return m_sensor.getAngle();
	}
	
	private void getPos() {
		m_pos = m_tracker.getPos();
	}
	
	public class PathConfig {
		public int m_points; 
		public double m_vel;
		public double m_accel;
		public double m_deccel;
		public double m_jerk;
		public double m_dt;
		public double m_wheelBase;
		
		public PathConfig(int points, double vel, double accel, double deccel, double jerk, double dt, double wheelBase) {
			m_points = points;
			m_vel = vel;
			m_accel = accel;
			m_deccel = deccel;
			m_jerk = jerk;
			m_dt = dt;
			m_wheelBase = wheelBase;
		}
	}
	
	public class VelocityContainer{
		public double leftVel, rightVel;
		
		public VelocityContainer(double leftVel, double rightVel) {
			this.leftVel = leftVel;
			this.rightVel = rightVel;
		}
	}
    
    private double ticksToFeet(double ticks) {
		return ticks / k_ticksFoot;
	}
	
	private double feetToTicks(double feet) {
		return feet * k_ticksFoot;
	}
}
