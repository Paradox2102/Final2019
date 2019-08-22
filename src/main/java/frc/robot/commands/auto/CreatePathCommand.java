package frc.robot.commands.auto;

import frc.robot.PurePursuit.PathConfig;
import frc.robotCore.Logger;
import frc.robot.Robot;
import frc.robot.Navigator.CameraDirection;
import edu.wpi.first.wpilibj.command.Command;
import frc.pathfinder.Pathfinder;
import frc.pathfinder.Pathfinder.Path;
import frc.pathfinder.Pathfinder.Waypoint;

public class CreatePathCommand extends Command {

	Path m_path;
	boolean m_isReversed;
	boolean m_setXY = true;
	boolean m_testMode;
    boolean m_isExtended;
    boolean m_cameraCorrection = false;
    CameraDirection m_cameraDirection;
    boolean m_stop = true;
    
    // private Field[] fields = {
    //     new Field("x", 'f'),
    //     new Field("y", 'f'),
    //     new Field("Target Apart", 'f')
    // };

    // CSVWriter writer = new CSVWriter("Target Calibration", fields);
	
    public CreatePathCommand(Waypoint[] points, PathConfig config, boolean isReversed, boolean setXY, boolean testMode) {
        requires(Robot.m_driveSubsystem);
        m_isReversed = isReversed;
        m_setXY = setXY;
        m_testMode = testMode;
        m_isExtended = true;
        m_stop = true;

        m_path = Pathfinder.computePath(points, config.m_points, config.m_dt, config.m_vel, config.m_accel, config.m_jerk, config.m_wheelBase);
    }

    public CreatePathCommand(Waypoint[] points, PathConfig config, boolean isReversed, boolean setXY, boolean testMode, boolean cameraCorrection, CameraDirection cameraDirection) {
        requires(Robot.m_driveSubsystem);
        m_isReversed = isReversed;
        m_setXY = setXY;
        m_testMode = testMode;
        m_isExtended = true;
        m_cameraCorrection = cameraCorrection;
        m_cameraDirection = cameraDirection;
        m_stop = true;

        m_path = Pathfinder.computePath(points, config.m_points, config.m_dt, config.m_vel, config.m_accel, config.m_jerk, config.m_wheelBase);
    }

    public CreatePathCommand(Waypoint[] points, PathConfig config, boolean isReversed, boolean setXY, boolean testMode, boolean cameraCorrection, CameraDirection cameraDirection, boolean stop) {
        requires(Robot.m_driveSubsystem);
        m_isReversed = isReversed;
        m_setXY = setXY;
        m_testMode = testMode;
        m_isExtended = true;
        m_cameraCorrection = cameraCorrection;
        m_cameraDirection = cameraDirection;
        m_stop = stop;

        m_path = Pathfinder.computePath(points, config.m_points, config.m_dt, config.m_vel, config.m_accel, config.m_jerk, config.m_wheelBase);
    }

    public CreatePathCommand(Waypoint[] points, PathConfig config, boolean isReversed, boolean setXY, boolean testMode, boolean cameraCorrection, boolean stop) {
        requires(Robot.m_driveSubsystem);
        m_isReversed = isReversed;
        m_setXY = setXY;
        m_testMode = testMode;
        m_isExtended = true;
        m_cameraCorrection = cameraCorrection;
        m_stop = stop;

        m_path = Pathfinder.computePath(points, config.m_points, config.m_dt, config.m_vel, config.m_accel, config.m_jerk, config.m_wheelBase);
    }
    // Called just before this Command runs the first time
    protected void initialize() {
//    	Robot.m_driveSubsystem.printPsath(m_path);
        // Robot.m_driveSubsystem.enablePID();
    	// if(m_testMode) {
    	// 	Robot.m_driveSubsystem.resetPreviousPosition();
        // }
        Robot.m_driveSubsystem.setCameraCorrection(m_cameraCorrection);

    	if(m_setXY) {
            Robot.m_driveSubsystem.setXY(m_path.m_centerPath[0].x, m_path.m_centerPath[0].y);
    	}
    	Robot.m_driveSubsystem.loadFollowPath(m_path, m_isReversed, m_isExtended);
        Robot.m_driveSubsystem.startFollow();
        
        // writer.start();
        Logger.Log("CreatePathCommand", 3, "Initialized");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(m_testMode) {
    		// SmartDashboard.putNumber("Left Encoder", Robot.m_driveSubsystem.getLeftPos());
    		// SmartDashboard.putNumber("Right Encoder", Robot.m_driveSubsystem.getRightPos());
        }

        // PositionContainer pos = Robot.m_driveSubsystem.getPos();
        
        // writer.write(pos.x, pos.y, Robot.navigator.getTargetPixelsApart());
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.m_driveSubsystem.isFollowFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.m_driveSubsystem.stopFollow();
        if(m_stop){
            Robot.m_driveSubsystem.stop();
        } //else{
        //     TeleopDriveCommand.m_disable = true;
        // }
    	if(m_testMode) {
    		// SmartDashboard.putNumber("Left Encoder", Robot.m_driveSubsystem.getLeftPos());
    		// SmartDashboard.putNumber("Right Encoder", Robot.m_driveSubsystem.getRightPos());
    	}
        // writer.finish();
        // System.out.println("End Create Path");
        Logger.Log("CreatePathCommand", 3, "End");
    }
}