package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SparkMaxWrapperVel implements PIDSource, PIDOutput{
    private CANSparkMax m_sparkMax;
    private CANEncoder m_encoder;
    private boolean m_invertEncoder;

    public SparkMaxWrapperVel(CANSparkMax sparkMax, CANEncoder encoder, boolean invertEncoder){
        m_sparkMax = sparkMax;
        m_encoder = encoder;
        m_invertEncoder = invertEncoder;
    }

    @Override
    public void pidWrite(double output) {
        m_sparkMax.set(output);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kRate;
    }

    @Override
    public double pidGet() {
        double velocity = m_encoder.getVelocity();

        return m_invertEncoder ? -velocity : velocity;
	}
} 