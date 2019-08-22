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

package frc.robotCore;

/**
 * @brief The Timer class provides a timer for measuring elapsed time
 * 
 */
public class Timer 
{
	private long m_zero = System.currentTimeMillis();
	
	/**
	 * @return - Returns the elapsed time in seconds
	 *
	 */
	public double get()
	{
		return((System.currentTimeMillis() - m_zero) * 0.001);
	}
	
	/**
	 * Resets the timer
	 *
	 */
	public void reset()
	{
		m_zero	= System.currentTimeMillis();
	}
	
	/**
	 * Causes the current thread to sleep
	 *
	 * @param seconds - specifies the delay time in seconds.
	 */
	static public void delay(double seconds)
	{
		try 
		{
			Thread.sleep((int) (seconds * 1000));
		} 
		catch (InterruptedException e) 
		{
			e.printStackTrace();
		}
	}
	
	/**
	 * Gets the current system time in seconds
	 *
	 * @return - Returns the current system time in seconds
	 */
	static public double getFPGATimestamp()
	{
		return(System.currentTimeMillis() * 0.001);
	}	
}
