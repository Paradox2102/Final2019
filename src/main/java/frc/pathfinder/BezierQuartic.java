package frc.pathfinder;

import frc.pathfinder.Bezier.BezierPoint;
import frc.pathfinder.Spline.SplinePoint;

public class BezierQuartic 
{
	BezierPoint m_p0, m_p1, m_p2, m_p3, m_p4;
	
	BezierQuartic(BezierPoint p0, BezierPoint p1, BezierPoint p2, BezierPoint p3, BezierPoint p4)
	{
		m_p0 = p0;
		m_p1 = p1;
		m_p2 = p2;
		m_p3 = p3;
		m_p4 = p4;
	}
	
	private void AdjustEndpoints(double angle1, double angle2, double scale)
	{
		double dx = m_p1.m_x - m_p0.m_x;
		double dy = m_p1.m_y - m_p0.m_y;
		double d = Math.sqrt(dx*dx + dy*dy) * scale;
//		double angle = Math.atan2(dy, dx);
		
		m_p1.m_x = m_p0.m_x + d * Math.cos(angle1);
		m_p1.m_y = m_p0.m_y + d * Math.sin(angle1);
		
		dx = m_p3.m_x - m_p4.m_x;
		dy = m_p3.m_y - m_p4.m_y;
		d = Math.sqrt(dx*dx + dy*dy);
		double angle = Math.atan2(dy, dx) * scale;
		
		m_p3.m_x = m_p4.m_x + d * Math.cos(angle2);
		m_p3.m_y = m_p4.m_y + d * Math.sin(angle2);
		
	}
	
	BezierQuartic(double p0x, double p0y, double angle1, double l1, double p3x, double p3y, double angle2, double l2)
	{
		double dx = l1 * Math.cos(angle1);
		double dy = l1 * Math.sin(angle1);

		BezierPoint p0 = new BezierPoint(p0x, p0y);
		BezierPoint p1 = new BezierPoint(p0x + dx, p0y + dy);
		
		dx = l2 * Math.cos(angle2);
		dy = l2 * Math.sin(angle2);
		
		BezierPoint p2 = new BezierPoint(p3x + dx, p3y + dy);
		BezierPoint p3 = new BezierPoint(p3x, p3y);
		
		CubicToQuartic(p0, p1, p2, p3);
		AdjustEndpoints(angle1, angle2, 2.5);
	}
	
	private void CubicToQuartic(BezierPoint p0, BezierPoint p1, BezierPoint p2, BezierPoint p3)
	{
		m_p0 = new BezierPoint(p0.m_x, p0.m_y);
		m_p1 = new BezierPoint(p0.m_x/4 + 3*p1.m_x/4, p0.m_y/4 + 3*p1.m_y/4);
		m_p2 = new BezierPoint(p1.m_x/2 + p2.m_x/2, p1.m_y/2 + p2.m_y/2);
		m_p3 = new BezierPoint(3*p2.m_x/4 + p3.m_x/4, 3*p2.m_y/4 + p3.m_y/4);
		m_p4 = new BezierPoint(p3.m_x, p2.m_y);
	}
	
    public void ComputeSplinePoints(SplinePoint[] points, int first, int sample_count, double maxVelocity) 
    {
        double distance = 0;
        
    	for (int i = 0 ; i <= sample_count ; i++)
    	{
    		double t = (double) i / sample_count;
    		double t2 = t * t;
    		double t3 = t2 * t;
    		double t4 = t3 * t;
    		double omt = (1 - t);
    		double omt2 = omt * omt;
    		double omt3 = omt2 * omt;
    		double omt4 = omt3 * omt;
    		
    		double dx = 4*omt3*(m_p1.m_x - m_p0.m_x) + 12*omt2*t*(m_p2.m_x - m_p1.m_x) + 12*omt*t2*(m_p3.m_x - m_p2.m_x) + 4*t3*(m_p4.m_x - m_p3.m_x)  ;
    		double dy = 4*omt3*(m_p1.m_y - m_p0.m_y) + 12*omt2*t*(m_p2.m_y - m_p1.m_y) + 12*omt*t2*(m_p3.m_y - m_p2.m_y) + 4*t3*(m_p4.m_y - m_p3.m_y)  ;
    		
    		points[first + i] = new SplinePoint();
    		points[first + i].m_heading = Math.atan2(dy, dx);
    		points[first + i].m_x = omt4*m_p0.m_x + 4*omt3*t*m_p1.m_x + 6*omt2*t2*m_p2.m_x + 4*omt*t3*m_p3.m_x + t4*m_p4.m_x;
    		points[first + i].m_y = omt4*m_p0.m_y + 4*omt3*t*m_p1.m_y + 6*omt2*t2*m_p2.m_y + 4*omt*t3*m_p3.m_y + t4*m_p4.m_y;
    		points[first + i].m_maxVelocity = maxVelocity;
    		
            if ((i+first) > 0)
            {
	            dx = points[i+first].m_x - points[i+first-1].m_x;
	            dy = points[i+first].m_y - points[i+first-1].m_y;
	            double delta = Math.sqrt(dx*dx + dy*dy);
	            
	            points[i+first].m_delta = delta;
            
	            distance = points[i+first-1].m_distance + delta;
            }
            
            points[i+first].m_distance = distance;
    	}
    }
}
