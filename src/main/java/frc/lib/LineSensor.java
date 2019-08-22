/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib;
/**
 * Add your docs here.
 */
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public class LineSensor {
    private final SPI m_port;

    public LineSensor(Port port){
        m_port = new SPI(port);
        m_port.setChipSelectActiveLow();
    }

    public final int[] getData(){
        byte[] cmd1 = new byte[] {1};
        byte[] cmd2 = new byte[] {0};
        
        byte[] ans1 = new byte[1];
        byte[] ans2 = new byte[1];
        byte[] ans3 = new byte[1];
        byte[] ans4 = new byte[1];
        byte[] ans5 = new byte[1];
        byte[] ans6 = new byte[1];
        byte[] ans7 = new byte[1];
        byte[] ans8 = new byte[1];
        byte[] ans9 = new byte[1];
        byte[] ans10 = new byte[1];

        m_port.transaction(cmd1, ans1, 1);
        m_port.transaction(cmd2, ans2, 1);
        m_port.transaction(cmd2, ans3, 1);
        m_port.transaction(cmd2, ans4, 1);
        m_port.transaction(cmd2, ans5, 1);
        m_port.transaction(cmd2, ans6, 1);
        m_port.transaction(cmd2, ans7, 1);
        m_port.transaction(cmd2, ans8, 1);
        m_port.transaction(cmd2, ans9, 1);
        m_port.transaction(cmd2, ans10, 1);

        int[] result = new int[8];

        result[0] = ans3[0] & 0xff;
        result[1] = ans4[0] & 0xff;
        result[2] = ans5[0] & 0xff;
        result[3] = ans6[0] & 0xff;
        result[4] = ans7[0] & 0xff;
        result[5] = ans8[0] & 0xff;
        result[6] = ans9[0] & 0xff;
        result[7] = ans10[0] & 0xff;

        return result;
    }
}
