package frc.lib;

import java.util.ArrayList;
import java.util.InputMismatchException;
import java.util.List;
import java.util.Scanner;

import frc.robot.Navigator.CameraDirection;

// import com.sun.tools.javac.jvm.Target;

import frc.robotCore.Logger;
import frc.robotCore.Network;

public class PiCamera implements Network.NetworkReceiver {
	public interface PiCameraAdvise
	{
		public void FrameReceived(CameraDirection direction);
	}
	private PiCameraAdvise m_advise = null;
	private CameraDirection m_direction = null;

	private final Network m_network = new Network();

	private final Object m_regionsLock = new Object();
	private List<TargetRegion> m_regions = new ArrayList<>();
	private List<TargetRegion> m_nextRegions = new ArrayList<>();

	private final Object m_frameDataLock = new Object();
	private int m_centerX;
	private int m_centerY;
	private int m_frameNo;

	private final double k_noTarget = 800.0;
	
	public class Rect {
		public int m_left;
		public int m_top;
		public int m_right;
		public int m_bottom;

		public Rect(int left, int top, int right, int bottom) {
			m_left = left;
			m_top = top;
			m_right = right;
			m_bottom = bottom;
		}

	}

	public enum TargetLeaningDirection
	{
		Left,
		Right,
		Ambiguous
	}

	public class TargetRegion {
		public int m_color;
		public int m_topLeft;
		public int m_topRight;
		public Rect m_bounds;
		public TargetLeaningDirection m_leaningDirection = TargetLeaningDirection.Ambiguous;

		public TargetRegion(int color, int left, int top, int right, int bottom, int topLeft, int topRight) {
			m_color = color;
			m_topLeft = topLeft;
			m_topRight = topRight;
			m_bounds = new Rect(left, top, right, bottom);

			setDirection();
		}

		private void setDirection(){
			int width;
			width = m_bounds.m_right - m_bounds.m_left;
			int height = m_bounds.m_bottom - m_bounds.m_top;
			int nPixels = width * height;
			int tWidth = width / 3;
			if (tWidth < 2)
			{
				tWidth = 2;
			}
			int tHeight = height / 3;
			if (tHeight < 4)
			{
				tHeight = 4;
			}
			int tPixels = tWidth * tHeight;
			int nTopLeft = m_topLeft;
			int nOn = m_topRight;
			
			double onTotalRatio = (double) nOn / (double) nPixels;
			double tOntTotalRatio = (double) nTopLeft / (double) tPixels;

			if (onTotalRatio > 0.75)
			{
				m_leaningDirection = TargetLeaningDirection.Ambiguous;
			}
			else if (tOntTotalRatio > 0.35)
			{
				m_leaningDirection = TargetLeaningDirection.Right;
			}
			else
			{
				m_leaningDirection = TargetLeaningDirection.Left;
			}
		}	
}

	public PiCamera(String host, int port, PiCameraAdvise advise, CameraDirection direction) {
		m_network.Connect(this, host, port);
		m_advise = advise;
		m_direction = direction;
	}

	private void processCameraFrame(String args) {
		Scanner scanner = new Scanner(args);

		try {
			synchronized (m_frameDataLock) {
				m_frameNo = scanner.nextInt();
				m_centerY = scanner.nextInt();
				m_centerX = scanner.nextInt();
			}
		} catch (InputMismatchException ex) {
			ex.printStackTrace();
		} finally {
			scanner.close();
		}
	}

	private void processCameraRegion(String args) {
		if (m_nextRegions != null) {
			Scanner scanner = new Scanner(args);

			try {
				m_nextRegions.add(new TargetRegion(scanner.nextInt(), scanner.nextInt(), scanner.nextInt(),
						scanner.nextInt(), scanner.nextInt(), scanner.nextInt(), scanner.nextInt()));
			} catch (InputMismatchException ex) {
				ex.printStackTrace();
			} finally {
				scanner.close();
			}
		}
	}
	
	long m_maxProcTime = 0;
	long m_avgProcTime = 0;
	long m_procCount = 0;

	private void processCameraEnd(String args) {
		synchronized (m_regionsLock) {
			m_regions = m_nextRegions;
			m_nextRegions = new ArrayList<>();
		}

//		if(m_direction == CameraDirection.Front){
//			int x = 0;
//		} else{
//			int y = 0;
//		}

		if (m_advise != null)
		{
			long time = System.currentTimeMillis();
			m_advise.FrameReceived(m_direction);
			time = System.currentTimeMillis() - time;
			
			if (time > m_maxProcTime)
			{
				m_maxProcTime = time;
			}
			
			m_avgProcTime += time;
			
			if (++m_procCount == 100)
			{
				System.out.println(String.format("avg=%f,max=%d", m_avgProcTime / (double) m_procCount, m_maxProcTime));
				m_avgProcTime = 0;
				m_maxProcTime = 0;
				m_procCount = 0;
			}
		}
	}

	// TODO: Immutable list
	public List<TargetRegion> getRegions() {
		synchronized (m_regionsLock) {
			return m_regions;
		}
	}

	public List<TargetRegion> sortRegions() {
		List<TargetRegion> regions = getRegions();
		if(regions.size() > 1){
			List<TargetRegion> sortedTargets = new ArrayList<TargetRegion>();
			sortedTargets.add(regions.get(0));

			for(int i=1; i<regions.size(); i++){
				int idx = sortedTargets.size();
				for(int j=sortedTargets.size()-1; j>-1; j--){
					if(regions.get(i).m_bounds.m_left < sortedTargets.get(j).m_bounds.m_left){
						idx = j;
					}
				}
				sortedTargets.add(idx, regions.get(i));
			}

			return sortedTargets;
		}else{
			return null;
		}
	}

	public double getTargetPixelsApartCalibration(){
		List<TargetRegion> regions = getRegions();

		if (regions.size() < 2) {
			return -1;
		} else{
			TargetRegion region1 = regions.get(0);
			TargetRegion region2 = regions.get(1);
			
			if (region1.m_bounds.m_left < region2.m_bounds.m_left) {
				return ((double) region2.m_bounds.m_left - region1.m_bounds.m_right);
			} else {
				return ((double) region1.m_bounds.m_left - region2.m_bounds.m_right);
			}
		}
	}

	static public double getTargetPixelsApart(TargetRegion[] regions){
		if(regions != null){
			TargetRegion region1 = regions[0];
			TargetRegion region2 = regions[1];
			
			if (region1.m_bounds.m_left < region2.m_bounds.m_left) {
				return ((double) region2.m_bounds.m_left - region1.m_bounds.m_right);
			} else {
				return ((double) region1.m_bounds.m_left - region2.m_bounds.m_right);
			}
		}
		return -1;
	}

	static public double getTargetCenter(TargetRegion[] regions){
		if(regions != null){
			TargetRegion region1 = regions[0];
			TargetRegion region2 = regions[1];
			if (region1.m_bounds.m_left < region2.m_bounds.m_left) {
				return ((double) region1.m_bounds.m_right + region2.m_bounds.m_left)/2.0;
			} else {
				return ((double) region1.m_bounds.m_left + region2.m_bounds.m_right)/2.0;
			}
		}
		return -1;
	}

	public double centerTargetDistanceFromCenter(TargetRegion[] regions){
		if(regions != null){
			TargetRegion region1 = regions[0];
			TargetRegion region2 = regions[1];
			
			if(region1.m_bounds.m_left < region2.m_bounds.m_left){
				return (double) (region1.m_bounds.m_right + region2.m_bounds.m_left)/2.0 - m_centerX;
			} else{
				return (double) (region2.m_bounds.m_right + region1.m_bounds.m_left)/2.0 - m_centerX;
			}
		}
		return -1;
	}

	public double leftTargetDistanceFromCenter(TargetRegion[] regions){
		if(regions != null){
			TargetRegion region1 = regions[0];
			TargetRegion region2 = regions[1];
			
			if(region1.m_bounds.m_left < region2.m_bounds.m_left){
				return (double) region1.m_bounds.m_right - m_centerX;
			} else{
				return (double) region2.m_bounds.m_right - m_centerX;
			}
		}
		return -1;
	}

	public enum TargetSelection{
		left, right, auto
	}

	public TargetRegion[] targetsClosestCenter(TargetSelection target){
		TargetRegion[] pair = new TargetRegion[2];
		int minDist = 1000;

		pair[0] = null;
		pair[1] = null;

		List<TargetRegion> regions = sortRegions();

		if(regions != null && regions.get(0) != null){
			if(target == TargetSelection.auto){
				int nTargets = regions.size();

				Logger.Log("PiCamera", -1, String.format("nTargets = %d", nTargets));

				for(int i = 0; i < nTargets - 1; i++){
					TargetRegion r1 = regions.get(i);
					TargetRegion r2 = regions.get(i+1);

					Logger.Log("PiCamera", -1, String.format("r1.right = %d, r2.left = %d", r1.m_bounds.m_right, r2.m_bounds.m_left));
		
					if( ((r1.m_leaningDirection == TargetLeaningDirection.Right) || (r2.m_leaningDirection == TargetLeaningDirection.Left)) && (r1.m_leaningDirection != r2.m_leaningDirection) ){
						int center = (r1.m_bounds.m_right + r2.m_bounds.m_left) / 2;
						int dist = Math.abs(center - m_centerX);
		
						if(dist < minDist){
							if(r1.m_bounds.m_top != 0 && r2.m_bounds.m_top != 0){
								pair[0] = r1;
								pair[1] = r2;
		
								minDist = dist;
							}
						}
					}
				}
			}else if(target == TargetSelection.left){
				int nTargets = regions.size();

				Logger.Log("PiCamera", -1, String.format("nTargets = %d", nTargets));

				if(nTargets > 2){
					int i=0;
					TargetRegion r1 = regions.get(i);
					TargetRegion r2 = regions.get(i+1);

					while(!(((r1.m_leaningDirection == TargetLeaningDirection.Right) || (r2.m_leaningDirection == TargetLeaningDirection.Left)) && (r1.m_leaningDirection != r2.m_leaningDirection) )){
						i += 1;
						if(i+1 < nTargets){
							r1 = regions.get(i);
							r2 = regions.get(i+1);
						}else{
							break;
						}	
					}

					if(r1.m_bounds.m_top != 0 && r2.m_bounds.m_top != 0){
						pair[0] = r1;
						pair[1] = r2;
					}
				}
			}else if(target == TargetSelection.right){
				int nTargets = regions.size();

				Logger.Log("PiCamera", -1, String.format("nTargets = %d", nTargets));

				if(nTargets > 2){
					int i=nTargets-1;
					TargetRegion r1 = regions.get(i-1);
					TargetRegion r2 = regions.get(i);

					while(!(((r1.m_leaningDirection == TargetLeaningDirection.Right) || (r2.m_leaningDirection == TargetLeaningDirection.Left)) && (r1.m_leaningDirection != r2.m_leaningDirection) )){
						i -= 1;
						if(i-1 >= 0){
							r1 = regions.get(i-1);
							r2 = regions.get(i);
						}else{
							break;
						}
					}

					if(r1.m_bounds.m_top != 0 && r2.m_bounds.m_top != 0){
						pair[0] = r1;
						pair[1] = r2;
					}
				}
			}
		}
		return pair;
	}

	public int getCenterX() {
		synchronized (m_frameDataLock) {
			return m_centerX;
		}
	}

	public int getCenterY() {
		synchronized (m_frameDataLock) {
			return m_centerY;
		}
	}

	long time = 0;
	long count = 0;

	@Override
	public void ProcessData(String data) {
		Logger.Log("PiCamera", -1, String.format("Data: %s", data));

		if (data == null) {
			synchronized (m_regionsLock) {
				m_regions = new ArrayList<>();
			}

			return;
		}

		if (time == 0)
		{
			time = System.currentTimeMillis();
		}

		switch (data.charAt(0)) {
		case 'F':
		
			if ((++count % 100) == 0)
			{
				System.out.println(String.format("rate = %f", (count * 1000.0) / (System.currentTimeMillis() - time)));
				count = 0;
				time = System.currentTimeMillis();
			}

			processCameraFrame(data.substring(1).trim());
			break;

		case 'R':
			// System.out.println("" + m_direction + ":" + data);
			processCameraRegion(data.substring(1).trim());
			break;

		case 'E':
			Logger.Log("PiCamera", -1, "# regions: " + m_nextRegions.size());
			processCameraEnd(data.substring(1).trim());
			break;

		default:
			Logger.Log("PiCamera", 3, String.format("Invalid command: %s", data));
			break;
		}
	}

}
