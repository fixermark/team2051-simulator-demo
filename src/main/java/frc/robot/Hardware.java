package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Abstraction class to hold references to all the robot's hardware (sensors, speed controllers, inputs)
 */
public class Hardware {
    private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(0);
    private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(1);

    public Hardware() {
        m_rightMotor.setInverted(true);
    }

    /**
     * Get the left motor
     * @return reference to left motor
     */
    public SpeedController leftMotor() {
        return m_leftMotor;
    }

    /**
     * Get the right motor
     * @return reference to right motor
     */
    public SpeedController rightMotor() {
        return m_rightMotor;
    }
}
