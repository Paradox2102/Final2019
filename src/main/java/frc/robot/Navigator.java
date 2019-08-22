package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.AutoPath;
import frc.lib.CSVWriter;
import frc.lib.LineSensor;
import frc.lib.PiCamera;
import frc.lib.AutoPath.AutoPathType;
import frc.lib.CSVWriter.Field;
import frc.lib.PiCamera.Rect;
import frc.lib.PiCamera.TargetLeaningDirection;
import frc.lib.PiCamera.TargetRegion;
import frc.lib.PiCamera.TargetSelection;
import frc.robot.PositionTracker.PositionContainer;
import frc.robot.commands.auto.LoadingZoneRocketCommand;
import frc.robot.commands.auto.RocketLoadingZoneCommand;
import frc.robot.commands.auto.autoTasks.ArmToFeederCommand;
import frc.robot.commands.auto.autoTasks.ClimbCommand;
import frc.robot.commands.auto.autoTasks.ClimbLevel2Command;
import frc.robot.commands.auto.autoTasks.ReleasePistonsLevel2Command;
import frc.robot.commands.auto.autoTasks.TeleopCargoShipPlacement;
import frc.robot.commands.climber.ReleaseCommand;
import frc.robot.commands.climber.ReleasePistonsDoubleCommand;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;
import frc.robot.commands.grabber.CloseGrabberCommand;
import frc.robot.commands.intakeWheels.OuttakeCargoCommand;
import frc.robotCore.Logger;
import frc.robotCore.Network;
import frc.robotCore.Network.NetworkReceiver;

public class Navigator implements PiCamera.PiCameraAdvise, NetworkReceiver
{
	private Object m_lock = new Object();
	private final PiCamera m_frontCamera = new PiCamera("10.21.2.11", 5800, this, CameraDirection.Front);
	private final PiCamera m_backCamera = new PiCamera("10.21.2.12", 5800, this, CameraDirection.Back);
	private Network m_network;
	private final LineSensor lineFollower = new LineSensor(SPI.Port.kOnboardCS0);

	private final int k_networkPort = 5803;

	private final double k_maxDistance = 13;
	private final double k_maxAngleError = DtoR(10);//was 5
	private final double k_angleRange = Math.PI/3.0;

	public ArrayList<AutoPath> m_autoPaths = new ArrayList<AutoPath>();

	public enum CameraDirection {
		Front, Back
	};

	public enum FieldSide {
		Right, Left
	};

	public enum CargoSide {
		Close, Middle, Far
	};

	public enum RocketSide {
		Close, Middle, Far
	};

	public Navigator(){
		createAutoPaths();
	}

	public static class RobotTarget
	{
		double m_x;
		double m_y;
		public double m_angle;
		
		RobotTarget(double x, double y, double angle)
		{
			m_x = x;
			m_y = y;
			m_angle = angle;
		}
	}

	public static RobotTarget[] m_targets = new RobotTarget[] { 
		new RobotTarget(-2.4, 25.28, DtoR(0)),      // A  0: Cargo ship far left side
		new RobotTarget(-2.4, 23.56, DtoR(0)),      // B  1: Cargo ship middle left side
		new RobotTarget(-2.4, 21.74, DtoR(0)),      // C  2: Cargo ship near left side
		new RobotTarget( 2.4, 25.28, DtoR(180)),    // D  3: Cargo ship far right side
		new RobotTarget( 2.4, 23.56, DtoR(180)),    // E  4: Cargo ship middle right side
		new RobotTarget( 2.4, 21.74, DtoR(180)),    // F  5: Cargo ship near right side
		new RobotTarget(-0.9, 18.35, DtoR(90)),     // G  6: Cargo ship left front
		new RobotTarget( 0.9, 18.35, DtoR(90)),     // H  7: Cargo ship right front
		new RobotTarget(-11.95, 17.82, DtoR(120)),  // I  8: Left rocket near hatch 
		new RobotTarget(-11.10, 19.14, DtoR(180)),  // J  9: Left rocket cargo
		new RobotTarget(-11.95, 20.42, DtoR(240)),  // K 10: Left rocket far hatch
		new RobotTarget( 11.95, 17.82, DtoR(60)),   // L 11: Right rocket near hatch
		new RobotTarget( 11.10, 19.14, DtoR(0)),    // M 12: Right rocket cargo
		new RobotTarget( 11.95, 20.42, DtoR(-60)),  // N 13: Right rocket far hatch
		new RobotTarget(-11.36,  0.00, DtoR(-90)),  // O 14: Left feeder
		new RobotTarget( 11.36,  0.00, DtoR(-90))   // P 15: Right feeder
	};

	public class Target{
		public RobotTarget m_target;
		public TargetRegion[] m_pair;
		public int m_targetIdx;

		public Target(RobotTarget target, TargetRegion[] pair, int targetIdx){
			m_target = target;
			m_pair = pair;
			m_targetIdx = targetIdx;
		}
	}
	
	private static double DtoR(double angle)
	{
		return(angle * Math.PI / 180);
	}

	public static double normalizeAngleRad(double angle){
		while(angle > Math.PI){
			angle -= Math.PI * 2;
		}

		while(angle < -Math.PI){
			angle += Math.PI * 2;
		}

		return angle;
	}

	public static double normalizeAngleDeg(double angle){
		while(angle > 180){
			angle -= 180 * 2;
		}

		while(angle < -180){
			angle += 180 * 2;
		}

		return angle;
	}

	public void initNetwork(){
		m_network = new Network();
		m_network.Listen(this, k_networkPort);
	}

	public void sendPositionData(PositionContainer pos){
		double angle = Robot.m_driveSubsystem.getAngle();
		m_network.SendMessage(String.format("p %.02f %.02f %.02f", pos.x, pos.y, angle));
	}

	public void sendCameraData(CameraDirection direction){
		//Getting front target data
		Target target = getCorrectTarget(direction);
		char camera = direction == CameraDirection.Front ? 'f' : 'b';
		
		if(target != null){
			Rect leftRect = target.m_pair[0].m_bounds;
			Rect rightRect = target.m_pair[1].m_bounds;
	
			m_network.SendMessage(String.format("t%c %d %d %d %d %d %d %d %d %d", camera, target.m_targetIdx, leftRect.m_left, leftRect.m_top, leftRect.m_right, leftRect.m_bottom,
								rightRect.m_left, rightRect.m_top, rightRect.m_right, rightRect.m_bottom));
		}else{
			m_network.SendMessage(String.format("t%cx", camera));
		}
	}

	public void sendCameraPos(CameraDirection direction){
		Target target = getCorrectTarget(direction);
		if(target != null){
			// double targetWidth = getTargetPixelsApart(target.m_pair, direction);
			// PositionContainer pos = getTargetXYDistance(targetWidth, target.m_pair, direction);
			PositionContainer pos = getCamPos();
			if(pos != null){
				// pos.x += target.m_target.m_x;
				// pos.y += target.m_target.m_y;
	
				char camera = direction == CameraDirection.Front ? 'f' : 'b';
				m_network.SendMessage(String.format("c%c %.02f %.02f", camera, pos.x, pos.y));
			}
		}
	}

	public void sendTimeRemaining(){
		double time = DriverStation.getInstance().getMatchTime();
		m_network.SendMessage(String.format("r %f", time));
	}

	public Target getCorrectTarget(CameraDirection direction)
	{
		synchronized (m_lock)
		{
			return (direction == CameraDirection.Front) ? m_lastFront : m_lastBack;
		}

	}

	public Target computeCorrectTarget(CameraDirection direction){
		double l = RobotMap.targetConstant / RobotMap.fixedTargetApart;
		double minError = 360;
		RobotTarget bestTarget = null;
		int bestTargetIdx = -1;
		TargetRegion[] bestPair = null;

		double robotAngle = Robot.m_driveSubsystem.getAngle();
		robotAngle += direction == CameraDirection.Front ? 0 : 180;
		robotAngle = Math.toRadians(robotAngle);

		PositionContainer robotPos = Robot.m_driveSubsystem.getPos();

		double cameraX = robotPos.x + RobotMap.k_chassisLength/2.0 * Math.cos(robotAngle);
		double cameraY = robotPos.y + RobotMap.k_chassisLength/2.0 * Math.sin(robotAngle);

		List<TargetRegion> sortedRegions = direction == CameraDirection.Front ? m_frontCamera.sortRegions() : m_backCamera.sortRegions();

		if(sortedRegions != null && sortedRegions.size() > 1){

			for(int t=0; t<m_targets.length; t++){
				RobotTarget target = m_targets[t];

				double dx = target.m_x - cameraX;
				double dy = target.m_y - cameraY;

				double theta = Math.atan2(dy, dx);
				double targetDistance = Math.sqrt(dy*dy + dx*dx);

				if((Math.abs(normalizeAngleRad(theta - robotAngle)) < k_angleRange) && (targetDistance < k_maxDistance) &&
						Math.abs(normalizeAngleRad(target.m_angle - robotAngle)) < k_angleRange){
					TargetRegion[] pair = new TargetRegion[2];

					for(int i=0; i<sortedRegions.size()-1; i++){
						if((sortedRegions.get(i).m_leaningDirection == TargetLeaningDirection.Right || 
							sortedRegions.get(i+1).m_leaningDirection == TargetLeaningDirection.Left) && 
							sortedRegions.get(i).m_leaningDirection != sortedRegions.get(i+1).m_leaningDirection){
							
							pair[0] = sortedRegions.get(i);
							pair[1] = sortedRegions.get(i+1);

							double offset = direction == CameraDirection.Front ? m_frontCamera.centerTargetDistanceFromCenter(pair) : m_backCamera.centerTargetDistanceFromCenter(pair);
							double thetaPrime = robotAngle - Math.atan(offset/l);
							double error = Math.abs(normalizeAngleRad(theta - thetaPrime));
							if(pair[0].m_bounds.m_top > 0 && pair[1].m_bounds.m_top > 0){
								if(error < k_maxAngleError)
								{
									if(error < minError){
										minError = error;
										bestTarget = target;
										bestTargetIdx = t;
										bestPair = new TargetRegion[] { pair[0], pair[1] } ;
									}
								}
							}
						}
					}
				}
			}
		}
		
		if(bestTarget != null){
			return new Target(bestTarget, bestPair, bestTargetIdx);
		}

		return null;
	}
	// public Target getCorrectTarget(CameraDirection direction){
	// 	double l = RobotMap.targetConstant / RobotMap.fixedTargetApart;
	// 	double minDistance = k_maxDistance;
	// 	RobotTarget bestTarget = null;
	// 	int bestTargetIdx = -1;
	// 	TargetRegion[] bestPair = null;

	// 	double robotAngle = Robot.m_driveSubsystem.getAngle();
	// 	robotAngle += direction == CameraDirection.Front ? 0 : 180;
	// 	robotAngle = Math.toRadians(robotAngle);

	// 	PositionContainer robotPos = Robot.m_driveSubsystem.getPos();

	// 	double cameraX = robotPos.x + RobotMap.k_chassisLength/2.0 * Math.cos(robotAngle);
	// 	double cameraY = robotPos.y + RobotMap.k_chassisLength/2.0 * Math.sin(robotAngle);

	// 	List<TargetRegion> sortedRegions = direction == CameraDirection.Front ? m_frontCamera.sortRegions() : m_backCamera.sortRegions();

	// 	if(sortedRegions != null && sortedRegions.size() > 1){

	// 		for(int t=0; t<m_targets.length; t++){
	// 			RobotTarget target = m_targets[t];

	// 			double dx = target.m_x - cameraX;
	// 			double dy = target.m_y - cameraY;

	// 			double theta = Math.atan2(dy, dx);
	// 			double targetDistance = Math.sqrt(dy*dy + dx*dx);

	// 			if((Math.abs(normalizeAngleRad(theta - robotAngle)) < k_angleRange) && (targetDistance < k_maxDistance)){
	// 				TargetRegion[] pair = new TargetRegion[2];

	// 				for(int i=0; i<sortedRegions.size()-1; i++){
	// 					pair[0] = sortedRegions.get(i);
	// 					pair[1] = sortedRegions.get(i+1);
			
	// 					double offset = m_frontCamera.centerTargetDistanceFromCenter(pair);
	// 					double thetaPrime = robotAngle - Math.atan(offset/l);
	// 					if(Math.abs(normalizeAngleRad(theta - thetaPrime)) < k_maxAngleError){
	// 						if(targetDistance < minDistance){
	// 							minDistance = targetDistance;
	// 							bestTarget = target;
	// 							bestTargetIdx = t;
	// 							bestPair = pair;
	// 						}
	// 						break;
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}
		
	// 	if(bestTarget != null){
	// 		return new Target(bestTarget, bestPair, bestTargetIdx);
	// 	}

	// 	return null;
	// }

	public double getTargetPixelsApartCalibration(CameraDirection direction){
		return direction == CameraDirection.Front ? m_frontCamera.getTargetPixelsApartCalibration() : m_backCamera.getTargetPixelsApartCalibration();
	}
	
	// public double getTargetPixelsApart(TargetRegion[] target, CameraDirection direction){
	// 	return direction == CameraDirection.Front ? m_frontCamera.getTargetPixelsApart(target) : m_backCamera.getTargetPixelsApart(target);
	// }

	public double getTargetCenterOffset(TargetRegion[] target, CameraDirection direction){
		return PiCamera.getTargetCenter(target) - (direction == CameraDirection.Front ? m_frontCamera.getCenterX() : m_backCamera.getCenterX());
	}

	public double getCameraAngleOffset(TargetRegion[] target, CameraDirection direction){
		if((target[0] == null) || (target[1] == null)){
			return Double.MAX_VALUE;
		}

		return getTargetAngleSimple(target, direction);
	}

	public double getCameraDistance(TargetRegion[] target, boolean earlyEnd, double endingPoint){
		if((target[0] == null) || (target[1] == null)){
			System.out.println("getCameraDistance: target null");
			Logger.Log("Get Camera Distance", 2, String.format("Null Target"));
			return Double.MAX_VALUE;
		}

		if((target[0].m_bounds.m_top < endingPoint || target[1].m_bounds.m_top < endingPoint) && earlyEnd){
			Logger.Log("Get Camera Distance", 2, String.format("Target Range Hit"));
			return Double.MAX_VALUE;
		}
		
		double targetWidth = PiCamera.getTargetPixelsApart(target);

		if(targetWidth == -1){
			Logger.Log("Get Camera Distance", 2, String.format("Null target Width"));
			return Double.MAX_VALUE;
		}

		if((targetWidth >= 143)){
			System.out.println("getCameraDistance: invalid width");
			Logger.Log("Get Camera Distance", 2, String.format("Invalid Width: %f", targetWidth));
			return Double.MAX_VALUE;
		}

		double distance = getTargetPolyDistance(targetWidth);

		return Math.sqrt(distance * distance - RobotMap.k_targetHeight * RobotMap.k_targetHeight);
	}

	public double getTargetPolyDistance(double targetWidth){
		return -1.246E-8*Math.pow(targetWidth, 4) - 2.0452E-6*(targetWidth*targetWidth*targetWidth) 
				+ 0.002036*(targetWidth*targetWidth) - 0.3356*targetWidth + 20.7055;
		// return 9.1079E-8*Math.pow(targetWidth, 4) - 4.499E-5*(targetWidth*targetWidth*targetWidth) 
		// 		+ 0.008499*(targetWidth*targetWidth) - 0.75759*targetWidth + 30.39166;
		// return 8.04551215137457E-08*Math.pow(targetWidth, 4) - 3.8957956132626E-05*(targetWidth*targetWidth*targetWidth) 
		// 		+ 0.007304521236486*(targetWidth*targetWidth) - 0.651453239816661*targetWidth + 27.1104614724312;
	}

	public double getTargetAngle(TargetRegion[] target, CameraDirection direction){
		double offset = CameraDirection.Front == direction ? m_frontCamera.leftTargetDistanceFromCenter(target) : m_backCamera.leftTargetDistanceFromCenter(target);
		double l = RobotMap.targetConstant / RobotMap.fixedTargetApart;
		
		return Math.toDegrees(Math.atan(offset/l)) - RobotMap.k_cameraFrontAngleOffset;
	}

	public TargetRegion[] targetsClosestCenter(CameraDirection direction, TargetSelection target){
		return CameraDirection.Front == direction ? m_frontCamera.targetsClosestCenter(target) : m_backCamera.targetsClosestCenter(target);
	}

	public double getTargetAngleSimple(TargetRegion[] target, CameraDirection direction){
		if(target[0] != null && target[1] != null){
			double offset = CameraDirection.Front == direction ? m_frontCamera.centerTargetDistanceFromCenter(target) : m_backCamera.centerTargetDistanceFromCenter(target);
			double l = RobotMap.targetConstant / RobotMap.fixedTargetApart;
			
			return Math.toDegrees(Math.atan(offset/l)) - (direction == CameraDirection.Front ? RobotMap.k_cameraFrontAngleOffset : RobotMap.k_cameraBackAngleOffset);
		}
		else{
			return Double.MAX_VALUE;	
		}
	}

	// TODO: describe variables 
	public double getTargetDistance(double targetWidth, TargetRegion[] target, CameraDirection direction){
		Target targetx = getCorrectTarget(direction);

		if(targetx != null){
			double theta = Math.toRadians(Robot.m_driveSubsystem.getAngle()) - targetx.m_target.m_angle;
			theta += direction == CameraDirection.Front ? 0 : Math.PI; // / 2.0;
	
			double offset = direction == CameraDirection.Front ? m_frontCamera.leftTargetDistanceFromCenter(target) : m_backCamera.leftTargetDistanceFromCenter(target);
	
			double l = RobotMap.targetConstant / RobotMap.fixedTargetApart;
	
			double alpha = Math.atan((offset + targetWidth)/l);
			double beta = Math.atan(offset/l);
			double omega = beta - theta + Math.toRadians(90);
	
			double dPrime = Math.sin(Math.PI - theta - omega) * targetWidth/Math.sin(omega);
			double lPrime = (dPrime/2.0) * Math.sin(omega) / Math.sin((alpha - beta)/2.0);
	
			double polyDist = getTargetPolyDistance(targetWidth);
			
			double distanceFromTarget = polyDist * (lPrime / dPrime) * (targetWidth/l);
	
			return Math.sqrt(distanceFromTarget*distanceFromTarget - RobotMap.k_targetHeight*RobotMap.k_targetHeight);
		}
		Logger.Log("Navigator", 1, "getTargetDistance fails");

		return(0); 
	}

	public PositionContainer getTargetXYDistance(double targetWidth, TargetRegion[] target, CameraDirection direction){
		double robotAngle;
		double offset;

		if(direction == CameraDirection.Front){
			robotAngle = Robot.m_driveSubsystem.getAngle();
			offset = m_frontCamera.leftTargetDistanceFromCenter(target);
		}else{
			robotAngle = Robot.m_driveSubsystem.getAngle() + 180;
			offset = m_backCamera.leftTargetDistanceFromCenter(target);
		}

		double theta = Math.toRadians(robotAngle - 90);

		double l = RobotMap.targetConstant / RobotMap.fixedTargetApart;

		double alpha = Math.atan((offset + targetWidth)/l);
		double beta = Math.atan(offset/l);
		double omega = beta - theta + Math.toRadians(90);
		
		double epsilon = Math.PI - omega;
		double rho = (Math.PI/2.0 - epsilon) + ((alpha - beta)/2.0);

		double hyp = getTargetDistance(targetWidth, target, direction);

		double cameraX = -Math.sin(rho) * hyp;
		double cameraY = -Math.cos(rho) * hyp;

		double xFromTarget = cameraX - RobotMap.k_cameraToCenterDist * Math.cos(Math.toRadians(robotAngle));
		double yFromTarget = cameraY - RobotMap.k_cameraToCenterDist * Math.sin(Math.toRadians(robotAngle));

		return new PositionContainer(xFromTarget, yFromTarget);
	}

	public boolean isTargetValid(CameraDirection direction){
		if(direction == CameraDirection.Front ? m_frontCamera.getRegions().size() >= 2 : m_backCamera.getRegions().size() >= 2){
			Target target = getCorrectTarget(direction);
		
			if(target == null){
				return false;
			}

			double pixelsApart = PiCamera.getTargetPixelsApart(target.m_pair);
			return pixelsApart <= 143 && pixelsApart >= 50;
		}
		return false;
	}

	private Target m_lastFront = null;
	private Target m_lastBack = null;

	private PositionContainer m_pos = new PositionContainer(0, 0);

	public PositionContainer getCamPos(){
		synchronized(m_lock){
			return new PositionContainer(m_pos.x, m_pos.y);
		}
	}
	Field[] k_fields = {
		new Field("Center Point", 'f')
	};
	CSVWriter m_writer = new CSVWriter("Camera Data", k_fields);

	public void startCameraWriter(){
		m_writer.start();
	}

	public void finishCameraWriter(){
		m_writer.finish();
	}

	@Override
	public void FrameReceived(CameraDirection direction) {
		// double targetWidth = getTargetPixelsApartCalibration(direction);
		Target target = computeCorrectTarget(direction);

		synchronized(m_lock)
		{
			if (direction == CameraDirection.Front)
			{
				m_lastFront = target;
			}
			else
			{
				m_lastBack = target;
				if((target != null) && target.m_pair[0] != null && target.m_pair[1] != null){
					// m_writer.write((target.m_pair[0].m_bounds.m_right + target.m_pair[1].m_bounds.m_left)/2.0);
				}else{
					m_writer.write(-1.0);
				}
			}
		}

		if(target != null){
			double targetWidth = PiCamera.getTargetPixelsApart(target.m_pair);
			m_pos = getTargetXYDistance(targetWidth, target.m_pair, direction);
			
			synchronized(m_lock){
				m_pos.x += target.m_target.m_x;
				m_pos.y += target.m_target.m_y;
			}

			// SmartDashboard.putNumber("Camera X:", m_pos.x);
			// SmartDashboard.putNumber("Camera Y", m_pos.y);

			Robot.m_driveSubsystem.cameraUpdate(m_pos, direction);
		}
	}

	public int[] lineFollower(){
		return lineFollower.getData();
	}

	public void createAutoPaths(){
		m_autoPaths.add(new AutoPath(AutoPathType.Level1, 15, 11, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level1)));
		m_autoPaths.add(new AutoPath(AutoPathType.Hatch, 11, 15, new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level1, true)));
		m_autoPaths.add(new AutoPath(AutoPathType.Cargo, 11, 15, new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level1, false)));
		m_autoPaths.add(new AutoPath(AutoPathType.Level2, 15, 11, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level2)));
		m_autoPaths.add(new AutoPath(AutoPathType.Level3, 15, 11, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Close, ElevatorLevel.Level3)));
		m_autoPaths.add(new AutoPath(AutoPathType.Level1, 15, 13, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Far, ElevatorLevel.Level1)));
		m_autoPaths.add(new AutoPath(AutoPathType.Hatch, 13, 15, new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Far, ElevatorLevel.Level1, true)));
		m_autoPaths.add(new AutoPath(AutoPathType.Cargo, 13, 15, new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Far, ElevatorLevel.Level1, false)));
		m_autoPaths.add(new AutoPath(AutoPathType.Level1, 15, 12, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Middle, ElevatorLevel.Level1)));
		m_autoPaths.add(new AutoPath(AutoPathType.Hatch, 12, 15, new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Middle, ElevatorLevel.Level1, true)));
		m_autoPaths.add(new AutoPath(AutoPathType.Cargo, 12, 15, new RocketLoadingZoneCommand(FieldSide.Right, RocketSide.Middle, ElevatorLevel.Level1, false)));
		m_autoPaths.add(new AutoPath(AutoPathType.Level2, 15, 12, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Middle, ElevatorLevel.Level2)));
		m_autoPaths.add(new AutoPath(AutoPathType.Level3, 15, 12, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Middle, ElevatorLevel.Level3)));
		m_autoPaths.add(new AutoPath(AutoPathType.Level2, 15, 13, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Far, ElevatorLevel.Level2)));
		m_autoPaths.add(new AutoPath(AutoPathType.Level3, 15, 13, new LoadingZoneRocketCommand(FieldSide.Right, RocketSide.Far, ElevatorLevel.Level3)));

	}

	private void Climb()
	{
		new ClimbCommand().start();
	}

	private void DoubleClimb(){
		new ClimbLevel2Command().start();
	}

	private void Release(){
		new ReleasePistonsLevel2Command().start();
	}

	private void ArmToFeeder(){
		new ArmToFeederCommand(true).start();
	}
	
	private void Outtake(){
		new OuttakeCargoCommand(1, 1).start();
	}

	private void EmergencyRelease(){
		new ReleasePistonsDoubleCommand().start();
	}

	private void CargoShipFront(){
		new TeleopCargoShipPlacement(true).start();
	}

	private void CargoShipBack(){
		new TeleopCargoShipPlacement(false).start();
	}

	private void CloseGrabber(){
		new CloseGrabberCommand().start();
	}

	private void AutoCommand(String command){
		AutoPathType type;
		int start = command.charAt(1) - 'A';
		int end = command.charAt(2) - 'A';
		switch (command.charAt(3)){
			case 'A':
				type = AutoPathType.Level1;
				break;
			case 'B':
				type = AutoPathType.Level2;
				break;
			case 'C':
				type = AutoPathType.Level3;
				break;
			case 'D':
				type = AutoPathType.Hatch;
				break;
			case 'E':
				type = AutoPathType.Cargo;
				break;
			default:
				type = AutoPathType.Invalid;
				break;
		}

		for (AutoPath path : m_autoPaths) {
			if(path.getType() == type && path.getStart() == start && path.getEnd() == end){
				path.getCommand().start();
				break;
			}
		}
	}

	@Override
	public void ProcessData(String command) 
	{
		System.out.println(command);

		switch (command.charAt(0))
		{
			case 'C':
				if(command.charAt(1) == '3'){
					Climb();
				}else if(command.charAt(1) =='2'){
					DoubleClimb();
				}
				
				break;
			case 'R':
				Release();
				break;
			case 'F':
				ArmToFeeder();
				break;
			case 'O':
				Outtake();
				break;
			case 'A':
				AutoCommand(command);
				break;
			case 'E':
				EmergencyRelease();
				break;
			case 'H':
				CargoShipFront();
				break;
			case 'B':
				CargoShipBack();
				break;
			case 'G':
				CloseGrabber();
				break;

		}
	}
}
