/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;
import frc.robotCore.LED5050String.RGBColor;

public class LEDBlinkCommand extends Command {
  boolean m_on = false;
  LEDSubsystem m_ledSubsystem;
  long m_lastCall = System.currentTimeMillis();
  double k_timeBetweenBlink = 1;
  int m_nLed;
  int m_red;
  int m_green;
  int m_blue;
  public LEDBlinkCommand(LEDSubsystem ledSubsystem, int red, int green, int blue) {
    requires(Robot.m_ledControlSubsystem);

    m_ledSubsystem = ledSubsystem;
    m_red = red;
    m_green = green;
    m_blue = blue;
    m_nLed = ledSubsystem.GetStringLength();
    setRunWhenDisabled(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    long curTime = System.currentTimeMillis();

    if(curTime - m_lastCall > k_timeBetweenBlink){
      m_on = !m_on;
    }

    setColors();

    m_lastCall = curTime;
  }

  private void setColors(){
    RGBColor[] colors = new RGBColor[m_nLed];
    if(m_on){
      for(int i=0; i<m_nLed; i++){
        colors[i] = new RGBColor(m_red, m_green, m_blue);
      }
    }else{
      for(int i=0; i<m_nLed; i++){
        colors[i] = new RGBColor(0, 0, 0);
      }
    }
    

    Robot.m_LEDSubsystem1.SetStringColors(colors);
    Robot.m_LEDSubsystem2.SetStringColors(colors);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_on = false;
    setColors();
  }
}
