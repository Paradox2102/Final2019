package frc.robot.triggers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Trigger;

public class POVChangeTrigger extends Trigger {
	private final GenericHID m_input;

	public POVChangeTrigger(final GenericHID input) {
		m_input = input;
	}

	@Override
	public boolean get() {
		if(m_input.getPOV() == -1){
			return false;
		}else{
			int pov = m_input.getPOV();
			pov = -pov + 180;
			return pov != 180;
		}
	}
}
