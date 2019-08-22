/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.lib.Condition;
import frc.robotCore.Logger;

public class ConditionCommand extends ConditionalCommand {
  Condition m_condition; 
  public ConditionCommand(Condition condition, final Command trueCommand, final Command falseCommand) {
    super(trueCommand, falseCommand);

    m_condition = condition;
  }

  protected boolean condition() {
    Logger.Log("Condition", 3, String.format("condition: %b", m_condition.condition()));
    return m_condition.condition();
  }
}
