package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class TalonWrapper implements PIDSource, PIDOutput{
    private TalonSRX m_talon;
    public TalonWrapper(TalonSRX talon){
        m_talon = talon;
    }

    @Override
    public void pidWrite(double output) {
        m_talon.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return m_talon.getSelectedSensorPosition();
	}

}