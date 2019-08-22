/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.MoveElevatorPosCommand.ElevatorLevel;

public class AutoPath {
    private AutoPathType m_type;
    private int m_start;
    private int m_end;
    private Command m_path;

    public enum AutoPathType {
        Level1, Level2, Level3, Hatch, Cargo, Invalid
    }

    public AutoPath(AutoPathType level, int start, int end, Command path){
        m_type = level;
        m_start = start;
        m_end = end;
        m_path = path;
    }

    public AutoPathType getType(){
        return m_type;
    }

    public int getStart(){
        return m_start;
    }

    public int getEnd(){
        return m_end;
    }

    public Command getCommand(){
        return m_path;
    }
}
