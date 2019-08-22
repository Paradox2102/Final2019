package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SparkMaxWrapperPos implements PIDSource, PIDOutput{
    private CANSparkMax m_sparkMax;
    private Encoder m_encoder;
    private double m_offset = 0;

    public SparkMaxWrapperPos(CANSparkMax sparkMax, Encoder encoder){
        m_sparkMax = sparkMax;
        m_encoder = encoder;
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
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return m_encoder.get() - m_offset;
    }
    
    public void setOffset(double offset){
        m_offset = offset;
    }
} 