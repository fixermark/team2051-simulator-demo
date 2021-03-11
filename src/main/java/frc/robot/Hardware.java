package frc.robot;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.sensors.RomiGyro;

/**
 * Abstraction class to hold references to all the robot's hardware (sensors, speed controllers, inputs)
 */
public class Hardware {
    /**
     * Number of encoder counts per full wheel revolution
     */
    public static final double ENCODER_TICKS_PER_REVOLUTION = 1440.0;

    /**
     * Circumference of wheel in meters
     */
    public static final double WHEEL_CIRCUMFERENCE_METERS = 0.07 * Math.PI;


    private final PWMVictorSPX m_leftMotor = new PWMVictorSPX(0);
    private final PWMVictorSPX m_rightMotor = new PWMVictorSPX(1);
    private final DigitalInput m_realRobot = new DigitalInput(8);
    private final RomiGyro m_gyro = new RomiGyro();
    private CounterBase m_leftEncoder = new Encoder(4, 5);
    private CounterBase m_rightEncoder = new Encoder(6,7);

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

    /**
     * Get left encoder count
     * @return left encoder count
     */
    public int leftEncoderCount() {
        return m_leftEncoder.get();
    }

    /**
     * Get right encoder count
     * @return right encoder count
     */
    public int rightEncoderCount() {
        return m_rightEncoder.get();
    }

    /**
     * Set encoders to use simulated encoders instead of rea
     * @param leftEncoder left simulated encoder
     * @param rightEncoder right simulated encoder
     */
    public void simulateEncoders(CounterBase leftEncoder, CounterBase rightEncoder) {
        m_leftEncoder = leftEncoder;
        m_rightEncoder = rightEncoder;
        m_rightMotor.setInverted(false);
    }

    /**
     * Get the gyro
     * @return reference to gyro
     */
    public RomiGyro gyro() {
        return m_gyro;
    }

    /**
     * Return 'true' if we believe this hardware is real (as opposed to simulated).
     * 
     * Romi runs on top of the simulator, so we can't use the RobotBase.isReal() method
     * to answer this question. Instead, we've bridged the EXT0 pin on the Romi to ground,
     * and we detect for a closed switch on the Romi. The simulator won't have the switch
     * closed, so we can differentiate.
     * 
     * NOTE: Do not call from robotInit(); inputs haven't been read by then.
     */
    public boolean isReal() {
        return !m_realRobot.get();
    }
}
