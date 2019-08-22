/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import java.util.Random;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;
import frc.robotCore.Timer;
import frc.robotCore.LED5050String.RGBColor;

public class LEDFireCommand extends Command {
	private double m_toggleTime;
	private int		   m_nLeds;
	private RGBColor[] m_colors;
	private LEDSubsystem m_subsystem;
	private RGBColor m_black = new RGBColor(0, 0, 0);
	private Timer m_timer = new Timer();
	
    public LEDFireCommand(LEDSubsystem subsystem, double toggleTime) 
    {
    	// Logger.Log("FireLightsCommand", 3, "FireLightsCommand()");
    	
    	m_subsystem = subsystem;
    	
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.m_ledControlSubsystem);
    	
    	m_nLeds = m_subsystem.GetStringLength();
    	m_colors = new RGBColor[m_nLeds];
      m_toggleTime = toggleTime;
      
      setRunWhenDisabled(true);
    }
    
    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	// Logger.Log("FireLightsCommand", 2, "initialize()");
    	
      m_timer.reset();
    }
    
    private int COOLING = 55;
    private int SPARKING = 120;
    private Random m_random = new Random();
    private boolean gReverseDirection = false;
    
    private int random8(int min, int max)
    {
    	return(min + m_random.nextInt(max - min));
    }
    
    private int random8(int max)
    {
    	return(m_random.nextInt(max));
    }
    
    private int random8()
    {
    	return(m_random.nextInt(255));
    }
    
    private int qsub8(int v1, int v2)
    {
    	int ret = v1 - v2;
    	
    	return((ret < 0) ? 0 : ret);
    }
    
    private int qadd8(int v1, int v2)
    {
    	int ret = v1 + v2;
    	
    	return((ret > 255) ? 255 : ret);
    }
    
    private int scale8_video( int i, int scale)
    {
        int j = ((i * scale) >> 8) + ((i != 0) && (scale != 0) ? 1 : 0);
        // uint8_t nonzeroscale = (scale != 0) ? 1 : 0;
        // uint8_t j = (i == 0) ? 0 : (((int)i * (int)(scale) ) >> 8) + nonzeroscale;
        return j;
    }
    
    private RGBColor HeatColor( int temperature)
    {
        RGBColor heatcolor = new RGBColor(0, 0, 0);
        
//        System.out.println(String.format("temp=%d", temperature));

        // Scale 'heat' down from 0-255 to 0-191,
        // which can then be easily divided into three
        // equal 'thirds' of 64 units each.
        int t192 = scale8_video( temperature, 192);

        // calculate a value that ramps up from
        // zero to 255 in each 'third' of the scale.
        int heatramp = t192 & 0x3F; // 0..63
        heatramp <<= 2; // scale up to 0..252

        // now figure out which third of the spectrum we're in:
        if((t192 & 0x80) != 0) {
            // we're in the hottest third
            heatcolor.m_red = 255; // full red
            heatcolor.m_green = 255; // full green
            heatcolor.m_blue = heatramp; // ramp up blue

        } else if( (t192 & 0x40) != 0 ) {
            // we're in the middle third
            heatcolor.m_red = 255; // full red
            heatcolor.m_green = heatramp; // ramp up green
            heatcolor.m_blue = 0; // no blue

        } else {
            // we're in the coolest third
            heatcolor.m_red = heatramp; // ramp up red
            heatcolor.m_green = 0; // no green
            heatcolor.m_blue = 0; // no blue
        }

        return heatcolor;
    }
    
    private int[] heat = null;
    
    private void Fire2012()
    {
    // Array of temperature readings at each simulation cell
    	if (heat == null)
    	{
    		 heat = new int[m_nLeds];
    	}

      // Step 1.  Cool down every cell a little
        for( int i = 0; i < m_nLeds; i++) {
          heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / m_nLeds) + 2));
//          System.out.println(String.format("h1[%d]=%d", i, heat[i]));
        }
      
        // Step 2.  Heat from each cell drifts 'up' and diffuses a little
        for( int k= m_nLeds - 1; k >= 2; k--) {
          heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
//          System.out.println(String.format("h2[%d]=%d", k, heat[k]));
        }
        
        // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
        if( random8() < SPARKING ) {
          int y = random8(7);
          heat[y] = qadd8( heat[y], random8(160,255) );
//          System.out.println(String.format("h3[%d]=%d", y, heat[y]));          
        }

        // Step 4.  Map from heat cells to LED colors
        for( int j = 0; j < m_nLeds; j++) {
          RGBColor color = HeatColor( heat[j]);
          int pixelnumber;
          if( gReverseDirection ) {
            pixelnumber = (m_nLeds-1) - j;
          } else {
            pixelnumber = j;
          }
          m_colors[pixelnumber] = color;
        }
    }
    
    private void printColors()
    {
    	for (int i = 0 ; i < m_nLeds ; i++)
    	{
    		// System.out.print(String.format("%02x%02x%02x ", m_colors[i].m_red, m_colors[i].m_green, m_colors[i].m_blue));
    	}
    	// System.out.println("");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	if (m_timer.get() > m_toggleTime)
    	{
    		m_timer.reset();
    		
    		Fire2012();
//    		printColors();
            Robot.m_LEDSubsystem1.SetStringColors(m_colors);
            Robot.m_LEDSubsystem2.SetStringColors(m_colors);
    	}
    }
    
    private void fill(RGBColor color)
    {
    	for (int i = 0 ; i < m_nLeds ; i++)
    	{
    		m_colors[i] = color;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	// Logger.Log("FireLightsCommand", -1, "isFinished()");
        
    	return (false);
    }

    // Called once after isFinished returns true
    protected void end() 
    {
    	// Logger.Log("FireLightsCommand", 2, "end()");
    	
    	fill(RGBColor.Black());
    	m_subsystem.SetStringColors(m_colors);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	// Logger.Log("FireLightsCommand", 2, "interrupted()");
    	end();
    }
}
