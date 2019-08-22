/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.LED.LEDControlCommand;
import frc.robotCore.LED5050String;
import frc.robotCore.LED5050String.RGBColor;

/**
 * Add your docs here.
 */
public class LEDSubsystem extends Subsystem {
	private LED5050String m_string;
	private boolean m_climbing;
	// private Command m_defCommand;
	
	public LEDSubsystem(int pin, int length)
	{
		// Logger.Log("LEDSubsystem", 3, "LEDSubsystem()");
		
		m_string = new LED5050String(pin, length);
	}
	
    public void initDefaultCommand() 
    {
		// setDefaultCommand(new LEDFireCommand(this, 0.02));
		// setDefaultCommand(new OuttakeLEDCommand(this, 0, 255, 0));
		setDefaultCommand(new LEDControlCommand(this));
    }
    
    public int GetStringLength()
    {
    	return(m_string.GetLength());
    }
    
    public void SetStringColors(RGBColor[] colors)
    {
    	m_string.SetColors(colors);
		m_string.Update();
	}
	
	public void setClimbing(boolean climbing){
		m_climbing = climbing;
	}

	public boolean getClimbing(){
		return m_climbing;
	}
}
