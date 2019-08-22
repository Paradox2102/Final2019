/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public class SPIGyro {
    private final SPI m_port;
    private double m_yawZero = 0;
    private int m_calls = 0;
    private int m_errors = 0;
    private int m_offset = 0;
    private double m_lastRaw = 0;
    private double m_angle;
    // private double m_angleRaw;
    private ReadThread m_thread;
    private Object lock = new Object();

    public SPIGyro(Port port){
        m_port = new SPI(port);
        m_port.setChipSelectActiveLow();
        m_thread = new ReadThread();

        m_thread.start();
        // m_port.setClockRate(100000);
    }

    public void resetYaw(double start) {
        synchronized(lock){
            m_offset = 0;
            m_yawZero = m_lastRaw + start;
        }
	}

    // private double getYawThread(){
    //     double yawRaw = getYawRawThread();
        
    //     // while(yawRaw == 2000){
    //     //     yawRaw = getYawRawThread();
    //     // }

    //     return (m_yawZero - yawRaw + m_offset);
    // }

    private void readYawRawThread()
    {
        double rawAngle = getAngle((byte) 1);
        if (Math.abs(m_lastRaw - rawAngle) > 180)
        {
            if (rawAngle > 0)
            {
                m_offset -= 360;
            }
            else
            {
                m_offset += 360;
            }
        }

        synchronized(lock)
        {
            m_lastRaw = rawAngle;
            m_angle = m_yawZero - rawAngle - m_offset;
        }
        
		// return rawAngle; // + m_offset;
	}

    private double getAngle(byte idx){
        byte[] cmd1 = new byte[] {idx};
        byte[] cmd2 = new byte[] {0};

        byte[] ans1 = new byte[1];
        byte[] ans2 = new byte[1];
        byte[] ans3 = new byte[1];
        byte[] ans4 = new byte[1];
        byte[] ans5 = new byte[1];

        while (true)
        {
            m_port.transaction(cmd1, ans1, 1);
            m_port.transaction(cmd2, ans2, 1);
            m_port.transaction(cmd2, ans3, 1);
            m_port.transaction(cmd2, ans4, 1);
            m_port.transaction(cmd2, ans5, 1);

            // System.out.println("getAngle");

            m_calls++;
            if (((ans5[0]) & 0xff) == ((ans3[0] + ans4[0]) & 0xff)){
                return ((ans3[0] & 0xff) | (ans4[0] << 8)) / 100.0;
            } else{
                // System.out.println("Gyro error!");
                m_errors++;
                // return(2000);
            }
        }
    }

    private class ReadThread extends Thread {
        public void run() {
			long step = 10;
			for(long nextRun = System.currentTimeMillis();;nextRun += step) {
                // double angle = getYawThread();
                // double angleRaw = 
                readYawRawThread();
                // synchronized(lock){
                //     m_angle = angleRaw + m_offset;
                //     m_angleRaw = angleRaw;
                // }
				try {
					long sleepTime = nextRun - System.currentTimeMillis();
					if(sleepTime > 0) {
						sleep(sleepTime);
					}
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
    }

    public double getYaw(){
        synchronized(lock){
            return m_angle;
        }
    }

    public double getYawRaw(){
        synchronized(lock){
            return m_lastRaw;
        }
    }
}
