package frc.robot;

import java.nio.channels.UnsupportedAddressTypeException;

import edu.wpi.first.wpilibj.CounterBase;

/**
 * An "encoder" that emits values injected into it from a simulator framework
 * (such as SimpleSimulatedChassis)
 */
public class SimulatedEncoder implements CounterBase {
    private int m_encoderZeroPoint = 0;
    private int m_encoderValue = 0;

    @Override
    public int get() {
        return m_encoderValue - m_encoderZeroPoint;
    }

    @Override
    public void reset() {
        m_encoderZeroPoint = m_encoderValue;
    }

    @Override
    public double getPeriod() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMaxPeriod(double maxPeriod) {
        throw new UnsupportedAddressTypeException();
    }

    @Override
    public boolean getStopped() {
        // simulated encoder never stops
        return false;
    }

    @Override
    public boolean getDirection() {
        // simulated encoder only supports one direction
        return false;
    }

    /**
     * Set the encoder
     * @param value New encoder value
     */
    public void set(int value) {
        m_encoderValue = value;
    }
}
