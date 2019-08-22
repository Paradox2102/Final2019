package frc.pathfinder;

import frc.pathfinder.Spline.SplinePoint;

public class Pathfinder 
{
	public static class Waypoint
	{
	    public double x, y, angle, maxVelocity;
	    public double l1 = 0;
	    public double l2 = 0;
	    public double l3 = 0;
	    public double l4 = 0;
	    
	    public Waypoint(double x_in, double y_in, double angle_in, double maxVelocity_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = maxVelocity_in;
	    	l1 = 0;
	    	l2 = 0;
	    }
	    
	    public Waypoint(double x_in, double y_in, double angle_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = 0;
	    	l1 = 0;
	    	l2 = 0;
	    }
	    
	    public Waypoint(double x_in, double y_in, double angle_in, double l1_in, double l2_in, double maxVelocity_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = maxVelocity_in;
	    	l1 = l1_in;
	    	l2 = l2_in;
	    }
	    
	    public Waypoint(double x_in, double y_in, double angle_in, double l1_in, double l2_in, double l3_in, double l4_in, double maxVelocity_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = maxVelocity_in;
	    	l1 = l1_in;
	    	l2 = l2_in;
	    	l3 = l3_in;
	    	l4 = l4_in;
	    }

	}

	public static class Coord
	{
	    public double x, y;
	    
	    public Coord(double x_in, double y_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    }
	}

	public static class Segment
	{
	    public double dt, x, y, position, velocity, acceleration, jerk, heading;
	    
	    public Segment()
	    {
	    	dt = 0;
	    	x = 0;
	    	y = 0;
	    	position = 0;
	    	velocity = 0;
	    	acceleration = 0;
	    	jerk = 0;
	    	heading = 0;
	    }
	    
	    public Segment(Segment seg)
	    {
	    	dt = seg.dt;
	    	x = seg.x;
	    	y = seg.y;
	    	position = seg.position;
	    	velocity = seg.velocity;
	    	acceleration = seg.acceleration;
	    	jerk = seg.jerk;
	    	heading = seg.heading;
	    }
	    
	    public Segment(	double dt_in,
	    				double x_in,
	    				double y_in,
	    				double position_in,
	    				double acceleration_in,
	    				double jerk_in,
	    				double heading_in)
	    {
	    	dt = dt_in;
	    	x = x_in;
	    	y = y_in;
	    	position = position_in;
	    	acceleration = acceleration_in;
	    	jerk = jerk_in;
	    	heading = heading_in;
	    }
	    
	    public static Segment[] CreateArray(int length)
	    {
	    	Segment[] segments = new Segment[length];
	    	
	    	for (int i = 0 ; i < length ; i++)
	    	{
	    		segments[i]	= new Segment();
	    	}
	    	
	    	return(segments);
	    }
	}
	
	public static class PathSpline
	{
		BezierQuintic[] m_bezierPoints;
		Spline[] m_splines;
		SplinePoint[] m_center;
		SplinePoint[] m_left;
		SplinePoint[] m_right;
		Segment[] m_centerSegments;
		Segment[] m_leftSegments;
		Segment[] m_rightSegments;
	}
	
	public static void computeDistance(SplinePoint[] traj, int idx)
	{
		if (idx > 0)
		{
			double dx	= traj[idx].m_x - traj[idx-1].m_x;
			double dy	= traj[idx].m_y - traj[idx-1].m_y;
			double d	= Math.sqrt(dx*dx + dy*dy);
			
			traj[idx].m_delta = d;
			traj[idx].m_distance = traj[idx-1].m_distance + d;
		}
	}
	
	public static void tankModify(SplinePoint[] original, SplinePoint[] left_traj, SplinePoint[] right_traj, int first, int count, double wheelbase_width) 
	{
	    double w = wheelbase_width / 2;
	    
	    for (int i = 0; i < count; i++) 
	    {
	        SplinePoint seg = original[i+first];
	        
	        double cos_angle = Math.cos(seg.m_heading);
	        double sin_angle = Math.sin(seg.m_heading);
	        	        
	        left_traj[i+first] = new SplinePoint(seg.m_x - (w * sin_angle), seg.m_y + (w * cos_angle), seg.m_heading, seg.m_maxVelocity);
	        right_traj[i+first] = new SplinePoint(seg.m_x + (w * sin_angle), seg.m_y - (w * cos_angle), seg.m_heading, seg.m_maxVelocity);
	        
	        computeDistance(left_traj, i+first);
	        computeDistance(right_traj, i+first);
	    }
	}
	
	@SuppressWarnings("unused")
	private static void printSplinePoints(SplinePoint[] points)
	{
		System.out.println("x,y,head,delta,dist");
        for (int j = 0 ; j < points.length ; j++)
        {
        	System.out.println(String.format("%f,%f,%f,%f,%f", points[j].m_x, points[j].m_y, 
        												MathUtil.normalizeDegrees(MathUtil.r2d(points[j].m_heading)),
        												points[j].m_delta, points[j].m_distance));
        }
		
	}
	
	public static int findPathPosition(SplinePoint[] points, double position, int idx)
	{
		while ((idx < points.length-1) && (position > points[idx].m_distance))
		{
			idx++;
		}
		
		return(idx);
	}
	
	public static int findPathPositionRev(SplinePoint[] points, double position, int idx)
	{
		while ((idx > 0) && (position < points[idx].m_distance))
		{
			idx--;
		}
		
		return(idx);
	}
	
	public static void addSegment(SplinePoint[] points, int idx, int lastIdx, double deltaPos, Segment[] segments, int segIdx, double dt, double d, boolean reverse, double startVelocity)
	{
		double x = points[lastIdx].m_x + (points[idx].m_x - points[lastIdx].m_x) * deltaPos;
		double y = points[lastIdx].m_y + (points[idx].m_y - points[lastIdx].m_y) * deltaPos;
		
		Segment segment = new Segment();
		
		segment.dt = dt;
		segment.x = x;
		segment.y = y;
		segment.heading = points[idx].m_heading;
		segment.velocity = d / dt;
		
		double dx;
		double dy;
		
		if (reverse)
		{
			dx = (segIdx < segments.length-1) ? segments[segIdx + 1].x - x : points[points.length-1].m_x - x;
			dy = (segIdx < segments.length-1) ? segments[segIdx + 1].y - y : points[points.length-1].m_y - y;
		}
		else
		{
			dx = (segIdx > 0) ? x - segments[segIdx - 1].x : x - points[0].m_x;
			dy = (segIdx > 0) ? y - segments[segIdx - 1].y : y - points[0].m_y;
		}
		double angle = Math.atan2(dy, dx);
		double da = MathUtil.normalizeRadians(angle - segment.heading);
		
		if (Math.abs(da) > Math.PI/2)
		{
			segment.velocity = -segment.velocity;
		}
		
		double dv;
		
		if (reverse)
		{
			if (segIdx < segments.length-1)
			{
				segment.position = segments[segIdx + 1].position - d;
				dv = segment.velocity - segments[segIdx + 1].velocity;
			}
			else
			{
				segment.position = points[points.length-1].m_distance - d;
				dv = segment.velocity - startVelocity;
			}
		}
		else
		{
			segment.position = (segIdx > 0) ? segments[segIdx - 1].position + d : d;
			dv = (segIdx > 0) ? segment.velocity - segments[segIdx - 1].velocity : segment.velocity;
		}
		
		segment.acceleration = dv / dt;
		
		if (reverse)
		{
			segment.acceleration = -segment.acceleration;
		}
		
		segments[segIdx]	= segment;
	}
	
	
	public static int computePath( SplinePoint[] left,
									SplinePoint[] center,
									SplinePoint[] right,
									Segment[] leftSeg,
									Segment[] centerSeg,
									Segment[] rightSeg,
									double dt,
									double max_velocity,
									double max_acceleration,
									double max_jerk,
									boolean reverse,
									double endPosition,
									double startVelocity)
	{
		int segIdx = reverse ? centerSeg.length - 1 : 0;
		double position = reverse ? center[center.length-1].m_distance : 0;
		double acceleration = 0;
		double velocity = startVelocity;
		int lastIdx = reverse ? center.length - 1 : 0;
		
		max_velocity = center[lastIdx].m_maxVelocity;

		double jerkSpeed = max_jerk != 0 ? max_velocity - 0.5*(max_acceleration * max_acceleration / max_jerk) : 0;
		
		while (reverse ? (lastIdx > 0) && (velocity < max_velocity) : (lastIdx < center.length-1) && (position < endPosition))
		{
			double nextVelocity = velocity;
			
			if (max_jerk > 0)
			{
				if (center[lastIdx].m_maxVelocity > max_velocity)
				{
					max_velocity = center[lastIdx].m_maxVelocity;
					jerkSpeed = max_velocity - 0.5*(max_acceleration * max_acceleration / max_jerk);
				}
				else if (center[lastIdx].m_maxVelocity < max_velocity)
				{
					max_velocity = center[lastIdx].m_maxVelocity;
					jerkSpeed = max_velocity + 0.5*(max_acceleration * max_acceleration / max_jerk);
				}
			}
			
			if (jerkSpeed != 0)
			{
				if (nextVelocity < max_velocity)
				{
					if (velocity >= jerkSpeed)
					{
						acceleration -= dt * max_jerk;
						
						if (acceleration < 0)
						{
							acceleration = 0;
							nextVelocity = max_velocity;
						}
						else
						{
							nextVelocity += dt * acceleration;
						}
					}
					else 
					{
						acceleration += dt * max_jerk;
						
						if (acceleration > max_acceleration)
						{
							acceleration = max_acceleration;
						}
						
						nextVelocity += dt * acceleration;
					}
					
					if (nextVelocity > max_velocity)
					{
						nextVelocity = max_velocity;
					}
				}
				else if (nextVelocity > max_velocity)
				{
//					if (nextVelocity < max_velocity)
					{
						if (velocity <= jerkSpeed)
						{
							acceleration += dt * max_jerk;
							
							if (acceleration > 0)
							{
								acceleration = 0;
								nextVelocity = max_velocity;
							}
							else
							{
								nextVelocity += dt * acceleration;
							}
						}
						else 
						{
							acceleration -= dt * max_jerk;
							
							if (acceleration < -max_acceleration)
							{
								acceleration = -max_acceleration;
							}
							
							nextVelocity += dt * acceleration;
						}
						
						if (nextVelocity < max_velocity)
						{
							nextVelocity = max_velocity;
						}
					}
				}
			}
			else
			{
				if (nextVelocity < max_velocity)
				{
					nextVelocity += dt * max_acceleration;
					
					if (nextVelocity > max_velocity)
					{
						nextVelocity = max_velocity;
					}

				}
				else if (nextVelocity > max_velocity)
				{
					nextVelocity -= dt * max_acceleration;
					
					if (nextVelocity < max_velocity)
					{
						nextVelocity = max_velocity;
					}
				}
			}
			
			
			double d = (dt * (velocity + nextVelocity) / 2);
			
			int idx;
			
			if (reverse)
			{
				idx = findPathPositionRev(center, position - d, lastIdx);
			}
			else
			{
				idx = findPathPosition(center, position + d, lastIdx);
			}
				
			if (idx == lastIdx)
			{
				if (reverse)
				{
					lastIdx++;
				}
				else
				{
					lastIdx--;
				}
			}
			
			double D = center[idx].m_distance - center[lastIdx].m_distance;
			double DL = left[idx].m_distance - left[lastIdx].m_distance;
			double DR = right[idx].m_distance - right[lastIdx].m_distance;
			double dl = d * DL / D;		// Distance left wheel moved
			double dr = d * DR / D;		// Distance right wheel moved
						
			/*
			 * If the left or right has moved too far, shorten the center to keep it inline
			 */
			if (dl > d)
			{
				dl = d;
				d  = dl * D / DL;
				dr = dl * DR / DL;
			}
			else if (dr > d)
			{
				dr = d;
				d  = dr * D / DR;
				dl = dr * DL / DR;
			}
			
			if (reverse)
			{
				position -= d;
			}
			else
			{
				position += d;
			}
			
			/*
			 * Compute percentage
			 */
			double deltaPos = (position - center[lastIdx].m_distance) / D;

			
			if ((segIdx < 0) || (segIdx > centerSeg.length))
			{
				throw new java.lang.ArrayIndexOutOfBoundsException("Insufficent segement space");
			}
			
			/*
			 * Create segments
			 */
			addSegment(center, idx, lastIdx, deltaPos, centerSeg, segIdx, dt, d, reverse, startVelocity);
			addSegment(left, idx, lastIdx, deltaPos, leftSeg, segIdx, dt, dl, reverse, startVelocity);
			addSegment(right, idx, lastIdx, deltaPos, rightSeg, segIdx, dt, dr, reverse, startVelocity);
			
			if (reverse)
			{
				segIdx--;
			}
			else
			{
				segIdx++;
			}
			velocity = nextVelocity;
			lastIdx = idx;

			if (reverse)
			{
				if (velocity == max_velocity)
				{
					break;
				}
			}
		}
		
//		printSegments(centerSeg, leftSeg, rightSeg, segIdx + 1);
		
		return(segIdx);
	}	
	
	public static void followPath(PathSpline path, double velocity, double dt, double max_velocity, 
																	double max_acceleration, double max_decl, double max_jerk, double finalVelocity)
	{
		/*
		 * Make a rough guess of how many segments we will need
		 */
		double length = path.m_center[path.m_center.length-1].m_distance;
		int count = (int) (5 * length / (max_velocity * dt)) + 1000;
		
		path.m_centerSegments = new Segment[count];
		path.m_leftSegments = new Segment[count];
		path.m_rightSegments = new Segment[count];
		
		/*
		 * First compute the ending deceleration segments
		 */
		Segment[] endCenter = new Segment[count];
		Segment[] endLeft = new Segment[count];
		Segment[] endRight = new Segment[count];
		
		int endSegIdx = computePath( 	path.m_left,
										path.m_center,
										path.m_right,
										endLeft,
										endCenter,
										endRight,
										dt,
										max_velocity,
										max_decl,
										max_jerk,
										true,
										0,
										finalVelocity) + 1;
		
		/*
		 * Now compute the starting segments up to the start of the deceleration period
		 */
		int segIdx = computePath( 	path.m_left,
									path.m_center,
									path.m_right,
									path.m_leftSegments,
									path.m_centerSegments,
									path.m_rightSegments,
									dt,
									max_velocity,
									max_acceleration,
									max_jerk,
									false,
									endCenter[endSegIdx].position,
									0);
		
		/*
		 * Now paste together the two sequences
		 */
		for (int s = endSegIdx ; s < endCenter.length ; s++, segIdx++)
		{
			path.m_centerSegments[segIdx] = endCenter[s];
			path.m_leftSegments[segIdx] = endLeft[s];
			path.m_rightSegments[segIdx] = endRight[s];
		}
		
		path.m_centerSegments = java.util.Arrays.copyOf(path.m_centerSegments, segIdx);
		path.m_leftSegments = java.util.Arrays.copyOf(path.m_leftSegments, segIdx);
		path.m_rightSegments = java.util.Arrays.copyOf(path.m_rightSegments, segIdx);
	}
	
	public static void printSegments(Segment[] center, Segment[] left, Segment[] right, int start)
	{
		System.out.println("t,dt,cx,cy,cp,dcp,cv,ca,lx,ly,lp,dlp,lv,la,rx,ry,rp,drp,rv,ra");
		
		double time = 0;
		
		for (int i = start ; (i < center.length) ; i++)
		{
			time += center[i].dt;
			
			double dcp = (i > start) ? center[i].position - center[i-1].position : 0;
			double dlp = (i > start) ? left[i].position - left[i-1].position : 0;
			double drp = (i > start) ? right[i].position - right[i-1].position : 0;
			
			System.out.println(String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
												time, center[i].dt, 
												center[i].x, center[i].y, center[i].position, dcp, center[i].velocity, center[i].acceleration,
												left[i].x, left[i].y, left[i].position, dlp, left[i].velocity, left[i].acceleration,
												right[i].x, right[i].y, right[i].position, drp, right[i].velocity, right[i].acceleration));
		}
		
	}
	
	public static class Path
	{
		public BezierQuintic[] m_bezierPoints;
		public Segment[] m_centerPath;
		public Segment[] m_leftPath;
		public Segment[] m_rightPath;
		
		public Path(BezierQuintic[] bezierPoints, Segment[] center, Segment[] left, Segment[] right)
		{
			m_bezierPoints = bezierPoints;
			m_centerPath = center;
			m_leftPath = left;
			m_rightPath = right;
		}
	}
	
	@SuppressWarnings("unused")
	private static void printSplinePoints(SplinePoint[] left, SplinePoint[] center, SplinePoint[] right)
	{
		System.out.println("lx,ly,ld,lp,cx,cy,cd,cp,rx,ry,rd,rp,a");
		
		for (int i = 0 ; i < center.length ; i++)
		{
			System.out.println(String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
								left[i].m_x, left[i].m_y, left[i].m_delta, left[i].m_distance,
								center[i].m_x, center[i].m_y, center[i].m_delta, center[i].m_distance,
								right[i].m_x, right[i].m_y, right[i].m_delta, right[i].m_distance,
								MathUtil.r2d(center[i].m_heading)));
		}
	}
	
	private static void fixupJoin(PathSpline path, int i, double dt, double maxAcceleration, double maxDV, double wheelBase)
	{
		while (i < path.m_center.length)
		{
			/*
			 * Compute the velocity needed for the right side to reach the next point in one tick
			 */
			double drx = path.m_rightSegments[i].x - path.m_rightSegments[i-1].x;
			double dry = path.m_rightSegments[i].y - path.m_rightSegments[i-1].y;
			double dr = Math.sqrt(drx*drx + dry*dry);
			double vr = dr / dt;
			
			/*
			 * Compute the maximum velocity allowable for the right side
			 */
			double v = path.m_rightSegments[i-1].velocity - (dt * maxAcceleration);
			
			if (v <= vr)
			{
				break;		// We are back on track
			}
			
			/*
			 * Compute the distance the right side will move
			 */
			dr = (path.m_rightSegments[i-1].velocity + v) * dt / 2;
			
			/*
			 * Compute average heading needed to get to the next point
			 */
			double dcx = path.m_centerSegments[i].x - path.m_centerSegments[i-1].x;
			double dcy = path.m_centerSegments[i].y - path.m_centerSegments[i-1].y;
			double avgAngle = Math.atan2(dcy, dcx);
			
			System.out.println(String.format("angle: %f==%f", MathUtil.r2d(avgAngle), MathUtil.r2d((path.m_centerSegments[i].heading+path.m_centerSegments[i-1].heading) / 2)));
			
			/*
			 * Compute the angle correction needed to maintain the right minimum distance
			 */
			double ddr = (dr - (path.m_rightSegments[i].position - path.m_rightSegments[i-1].position));	// Amount extra distance needed on right
			double da = ddr / wheelBase;																	// Angle correction needed to get extra distance
			avgAngle += da;																					// New average angle required
			double endAngle = (2 * avgAngle) - path.m_centerSegments[i].heading;							// New ending angle
			
			/*
			 * Compute the distance left side will move in the next tick
			 */
			double dlx = path.m_leftSegments[i].x - path.m_leftSegments[i-1].x;
			double dly = path.m_leftSegments[i].y - path.m_leftSegments[i-1].y;
			double dl = Math.sqrt(dlx*dlx + dly*dly); 
					
			System.out.println(String.format("left distance: %f==%f", dl, path.m_leftSegments[i].position - path.m_leftSegments[i-1].position));
			
			/*
			 * Compute the new position for the center of the track
			 */
			double d = (dl + dr) / 2;			// Distance center moves
			double cx = path.m_centerSegments[i-1].x + d * Math.cos(avgAngle);
			double cy = path.m_centerSegments[i-1].y + d * Math.sin(avgAngle);
			
			/*
			 * Compute the new positions for the left and right wheels
			 */
			double dx = wheelBase * Math.cos(Math.PI/2 - endAngle) / 2;
			double dy = wheelBase * Math.sin(Math.PI/2 - endAngle) / 2;
			double rx = cx + dx;
			double ry = cy - dy;
			double lx = cx - dx;
			double ly = cy + dy;
			
			/*
			 * Compute the distance the left side moves
			 */
			dlx = lx - path.m_leftSegments[i-1].x;
			dly = ly - path.m_leftSegments[i-1].y;
			double dln = Math.sqrt(dlx*dlx + dly*dly);
			
			/*
			 * Compute the distance the right side moves
			 */
			drx = rx - path.m_rightSegments[i-1].x;
			dry = ry - path.m_rightSegments[i-1].y;
			double drn = Math.sqrt(drx*drx + dry*dry);
			
			System.out.println(String.format("Left dist: %f->%f", dl, dln));
			System.out.println(String.format("Right dist: %f->%f",dr, drn));
			
			
			System.out.println(String.format("left: (%f,%f)->(%f,%f):(%f,%f)", 
						path.m_leftSegments[i].x, path.m_leftSegments[i].y, lx, ly, lx - path.m_leftSegments[i].x, ly - path.m_leftSegments[i].y));
			System.out.println(String.format("right: (%f,%f)->(%f,%f):(%f,%f)", 
					path.m_rightSegments[i].x, path.m_rightSegments[i].y, rx, ry, rx - path.m_rightSegments[i].x, ry - path.m_rightSegments[i].y));
			
			path.m_centerSegments[i].x = cx;
			path.m_centerSegments[i].y = cy;
			path.m_centerSegments[i].heading = endAngle;
			path.m_centerSegments[i].position = path.m_centerSegments[i-1].position + d;
			path.m_centerSegments[i].velocity = d / dt;
			
			path.m_leftSegments[i].x = lx;
			path.m_leftSegments[i].y = ly;
			path.m_leftSegments[i].heading = endAngle;
			path.m_leftSegments[i].position = path.m_leftSegments[i-1].position + d;
			path.m_leftSegments[i].velocity = dl / dt;
			
			path.m_rightSegments[i].x = rx;
			path.m_rightSegments[i].y = ry;
			path.m_rightSegments[i].heading = endAngle;
			path.m_rightSegments[i].position = path.m_rightSegments[i-1].position + d;
			path.m_rightSegments[i].velocity = dr / dt;
			
			int xx = 0;	
		}
	}
	
	private static void fixupPath(PathSpline path, double dt, double maxAcceleration, double wheelBase)
	{
		double maxDV = 1.1 * dt * maxAcceleration;	// Give is some margin
		
		for (int i = 1 ; i < path.m_centerSegments.length ; i++)
		{
			double dv = path.m_rightSegments[i].velocity - path.m_rightSegments[i-1].velocity;
			
			if (Math.abs(dv) > maxDV)
			{
				fixupJoin(path, i, dt, maxAcceleration, maxDV, wheelBase);
				break;
			}
			
		}
	}
	
	public static SplinePoint[] computeBezier(	final Waypoint[] path, int sample_count)
	{
	    if (path.length < 2) return(null);
	    
	    SplinePoint[] centerSpline = new SplinePoint[(path.length-1) * (sample_count)];
	    
	    for (int i = 0 ; i < path.length - 1 ; i++) 
	    {
	        Spline s = new Spline();
	        Hermite.fit_Quintic(path[i], path[i+1], s);
	        
        	BezierQuintic bezier = new BezierQuintic(path[i].x, path[i].y, path[i].angle, path[i].l1, path[i].l3,
        								path[i+1].x, path[i+1].y, path[i+1].angle + Math.PI, path[i].l2, path[i].l4);
        	
        	bezier.ComputeSplinePoints(centerSpline,  i * (sample_count), sample_count, path[i].maxVelocity);
	    }
	    
	    return centerSpline;
	}
		
	public static Path computePath(	final Waypoint[] path_in, 
			int sample_count, 
			double dt,
			double max_velocity, 
			double max_acceleration, 
			double max_jerk,
			double wheelBase)
	{
		return(computePath(path_in, sample_count, dt, max_velocity, max_acceleration, max_acceleration, max_jerk, wheelBase));
	}
	
	public static Path computePath(	final Waypoint[] path_in, 
									int sample_count, 
									double dt,
									double max_velocity, 
									double max_acceleration, 
									double max_decl,
									double max_jerk,
									double wheelBase)
	{
	    if (path_in.length < 2) return(null);
	    
	    Waypoint[] path = new Waypoint[path_in.length];
	    
	    for (int i = 0 ; i < path_in.length ; i++)
	    {
	    	path[i] = new Waypoint(path_in[i].x, path_in[i].y, path_in[i].angle, path_in[i].l1, path_in[i].l2, path_in[i].l3, path_in[i].l4, path_in[i].maxVelocity > 0 ? path_in[i].maxVelocity : max_velocity);
	    }
	    
	    PathSpline pathSpline = new PathSpline();
	    
//	    pathSpline.m_waypoints = path;
	    
	    SplinePoint[] centerSpline = new SplinePoint[(path.length-1) * (sample_count)];
	    SplinePoint[] leftSpline = new SplinePoint[centerSpline.length];
	    SplinePoint[] rightSpline = new SplinePoint[centerSpline.length];
	    
	    pathSpline.m_center = centerSpline;
	    pathSpline.m_left = leftSpline;
	    pathSpline.m_right = rightSpline;
	    pathSpline.m_bezierPoints = new BezierQuintic[path.length - 1];
	    
	    for (int i = 0 ; i < path.length - 1 ; i++) 
	    {
	        Spline s = new Spline();
	        Hermite.fit_Quintic(path[i], path[i+1], s);
	        
        	BezierQuintic bezier = new BezierQuintic(path[i].x, path[i].y, path[i].angle, path[i].l1, path[i].l3,
        								path[i+1].x, path[i+1].y, path[i+1].angle + Math.PI, path[i].l2, path[i].l4);
        	
        	bezier.ComputeSplinePoints(centerSpline,  i * (sample_count), sample_count, path[i].maxVelocity);
        	
        	pathSpline.m_bezierPoints[i] = bezier;
	        
	    	tankModify(centerSpline, leftSpline, rightSpline, i * (sample_count), sample_count, wheelBase); 
	    	

	    }
	    
//	      printSplinePoints(leftSpline, centerSpline, rightSpline);
//        printSplinePoints(centerSpline);
//        printSplinePoints(leftSpline);
//        printSplinePoints(rightSpline);
        
    	followPath(pathSpline, 0, dt, max_velocity, max_acceleration, max_decl, max_jerk, path_in[path_in.length-1].maxVelocity);
//    	fixupPath(pathSpline, dt, max_acceleration, wheelBase);
    	
//    	printSegments(pathSpline.m_centerSegments, pathSpline.m_leftSegments, pathSpline.m_rightSegments, 0);
    	
    	return(new Path(pathSpline.m_bezierPoints, pathSpline.m_centerSegments, pathSpline.m_leftSegments, pathSpline.m_rightSegments));
	}
}
