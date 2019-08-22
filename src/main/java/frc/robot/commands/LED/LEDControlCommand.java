/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSubsystem;
import frc.robotCore.LED5050String.RGBColor;

public class LEDControlCommand extends Command {
  LEDSubsystem m_ledSubsystem;
  int m_nLed;
  long m_lastCall = 0;
  double k_timeBetweenBlink = 1;
  boolean red;
  static LedState m_ledState;

  private enum LedState {
    intake, outtake, climb, fire, deploy
  };

  public LEDControlCommand(LEDSubsystem ledSubsystem) {
    requires(ledSubsystem);
    m_ledSubsystem = ledSubsystem;
    m_nLed = ledSubsystem.GetStringLength();

    if(DriverStation.getInstance().getAlliance() == Alliance.Red){
      red = true;
    }else{
      red = false;
    }
    setRunWhenDisabled(true);

    // m_ledState = LedState.fire;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println("called");
    if(System.currentTimeMillis() - m_lastCall > 5000){
      // if(Robot.m_ledControlSubsystem.getCurrentCommand() == null){
      //   m_ledState = LedState.fire;
      // }

      if(Robot.m_oi.getElevatorButton(6)){//outtake
        if(m_ledState != LedState.outtake){
          if(Robot.m_ledControlSubsystem.getCurrentCommand() != null){
            Robot.m_ledControlSubsystem.getCurrentCommand().cancel();
          }
        }
        // System.out.println("outtake");
        m_ledState = LedState.outtake;
        setColors(new RGBColor(100, 0, 100));
      }else if(Robot.m_oi.getElevatorButton(1)){//intake
        if(m_ledState != LedState.intake){
          if(Robot.m_ledControlSubsystem.getCurrentCommand() != null){
            Robot.m_ledControlSubsystem.getCurrentCommand().cancel();
          }
        }
        m_ledState = LedState.intake;
        setColors(new RGBColor(0, 0, 255));
      }else if(m_ledSubsystem.getClimbing()){
        if(m_ledState != LedState.climb){
          if(Robot.m_ledControlSubsystem.getCurrentCommand() != null){
            Robot.m_ledControlSubsystem.getCurrentCommand().cancel();
          }
  
          // RGBColor color;
          // if(red){
          //   color = new RGBColor(255, 0, 0);
          // }else{
          //   color = new RGBColor(0,0,255);
          // }
          new LedClimbCommand().start();
        }  
        m_ledState = LedState.climb;
      }else if(Robot.m_oi.getElevatorButton(5) || Robot.m_oi.getElevatorButton(3) || Robot.m_oi.getElevatorButton(4)){
        if(m_ledState != LedState.deploy){
          if(Robot.m_ledControlSubsystem.getCurrentCommand() != null){
            Robot.m_ledControlSubsystem.getCurrentCommand().cancel();
          }
          new LEDCargoDeployCommand(5).start();
          m_lastCall = System.currentTimeMillis();
        }
        m_ledState = LedState.deploy;
      }else{
        if(m_ledState != LedState.fire){
          if(Robot.m_ledControlSubsystem.getCurrentCommand() != null){
            Robot.m_ledControlSubsystem.getCurrentCommand().cancel();
          }
          new LEDFireCommand(m_ledSubsystem, 0.02).start();
        }
        // System.out.println("FIRE");
        m_ledState = LedState.fire;
      }
    }
    
  }

  private void setColors(RGBColor color){
    RGBColor[] colors1 = new RGBColor[m_nLed];
    RGBColor[] colors2 = new RGBColor[m_nLed];

    for(int i=0; i<m_nLed; i++){
      colors1[i] = new RGBColor(color.m_red, color.m_green, color.m_blue);
      colors2[i] = new RGBColor(color.m_red, color.m_green, color.m_blue);
    }
    Robot.m_LEDSubsystem1.SetStringColors(colors1);
    Robot.m_LEDSubsystem2.SetStringColors(colors2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }
}
