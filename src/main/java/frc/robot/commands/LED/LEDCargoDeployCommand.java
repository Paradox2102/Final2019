/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robotCore.LED5050String.RGBColor;

public class LEDCargoDeployCommand extends Command {
  int m_nLed;
  public LEDCargoDeployCommand(int time) {
    requires(Robot.m_ledControlSubsystem);

    m_nLed = Robot.m_LEDSubsystem1.GetStringLength();

    setTimeout(time);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    RGBColor[] colors = new RGBColor[m_nLed];

    for(int i=0; i<m_nLed; i++){
      colors[i] = new RGBColor((int)(Math.random()*120), (int)(Math.random()*256), (int)(Math.random()*256));
    }

    setColors(colors);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  private void setColors(RGBColor[] colors){
    Robot.m_LEDSubsystem1.SetStringColors(colors);
    Robot.m_LEDSubsystem2.SetStringColors(colors);
  }
}
