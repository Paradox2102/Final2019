/*
 *	  Copyright (C) 2016  John H. Gaby
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, version 3 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *    
 *    Contact: robotics@gabysoft.com
 */

package frc.robot.commands.LED;

import frc.robotCore.LED5050String.RGBColor;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;
import frc.robotCore.Logger;
import frc.robotCore.Timer;

/**
 *
 */
public class MovingLights extends Command 
{
	private double m_toggleTime;
	private int		   m_nLeds;
	private RGBColor[] m_colors;
	private LEDSubsystem m_subsystem;
	private RGBColor m_color;
	private int m_nMovingLights;

	private Timer m_timer = new Timer();
	private int m_lightOn;
	
    public MovingLights(LEDSubsystem subsystem, RGBColor color, int nMovingLights, double toggleTime) 
    {
    	Logger.Log("MovingLights", 3, "MovingLights()");
    	
    	m_subsystem = subsystem;
    	
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.m_ledControlSubsystem);
    	
    	m_nLeds = m_subsystem.GetStringLength();
    	m_colors = new RGBColor[m_nLeds];
    	m_color = color;
    	m_nMovingLights = nMovingLights;
    	m_toggleTime = toggleTime;
    	
    	for (int i = 0 ; i < m_nLeds ; i++)
    	{
    		m_colors[i] = new RGBColor(0, 0, 0);
    	}
    	
    }

	// Called just before this Command runs the first time
    protected void initialize() 
    {
    	Logger.Log("MovingLights", 2, "initialize()");
    	m_lightOn = 0;
    	for(int i=0;i<m_nMovingLights;i++)
    	{
    		m_colors[i] = m_color;
    	}
    	m_subsystem.SetStringColors(m_colors);
    	m_timer.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	if (m_timer.get() > m_toggleTime)
    	{
    		for(int i =0; i<m_nMovingLights;i++)
    		{
    			m_colors[m_lightOn+i] = new RGBColor(0, 0, 0);
    		}
    		
    		m_lightOn += 2;
    		
    		if(m_lightOn>=m_nLeds-m_nMovingLights+1)
    		{
    			m_lightOn=0;
    		}
    		
    		for(int i =0; i<m_nMovingLights;i++)
    		{
    			m_colors[m_lightOn+i] = m_color;
    		}
    		
			Robot.m_LEDSubsystem1.SetStringColors(m_colors);
			Robot.m_LEDSubsystem2.SetStringColors(m_colors);
        	m_timer.reset();
    	}

    }
    
    private void fill(int r, int g, int b)
    {
    	for (int i = 0 ; i < m_nLeds ; i++)
    	{
    		m_colors[i] = new RGBColor(0, 0, 0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	Logger.Log("MovingLights", -1, "isFinished()");
        
    	return (false);
    }

    // Called once after isFinished returns true
    protected void end() 
    {
    	Logger.Log("MovingLights", 2, "end()");
    	
    	fill(0, 0, 0);
    	m_subsystem.SetStringColors(m_colors);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	Logger.Log("MovingLights", 2, "interrupted()");
    	end();
    }
}
