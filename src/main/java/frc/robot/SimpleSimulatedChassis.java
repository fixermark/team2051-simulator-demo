package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.RomiGyro;

/**
 * A simulation of the robot's chassis, converting input drive voltages to
 * changes to a robot pose
 */
public class SimpleSimulatedChassis {
    private RomiGyro m_gyro;
    private SimulatedEncoder m_leftEncoder;
    private SimulatedEncoder m_rightEncoder;
    private double m_leftEncoderValue = 0;
    private double m_rightEncoderValue = 0;
    private double m_rotation = 0;

    /**
     * Speed at which robot drives forward in meters per second at max velocity
     */
    private static final double FORWARD_SPEED_MS = 1.0;

    /**
     * Number of encoder counts per full wheel revolution
     */
    private static final double ENCODER_TICKS_PER_REVOLUTION = 1440.0;

    /**
     * Circumference of wheel in meters
     */
    private static final double WHEEL_CIRCUMFERENCE_METERS = 0.07 * Math.PI;

    /**
     * Ticks per second at full speed
     */
    private static final double TICKS_PER_SECOND = FORWARD_SPEED_MS / WHEEL_CIRCUMFERENCE_METERS * ENCODER_TICKS_PER_REVOLUTION;

    /**
     * Rotations per second at full forward + reverse
     * 
     * We could probably do some trigonometry to estimate this by considering how far
     * one wheel or the other turns and conclude what the rotation must be as a result,
     * but easier to make up a number
     */
    private static final double ROTATIONS_PER_SECOND = 2.0;

    private double m_lastUpdateTime;

    public SimpleSimulatedChassis (RomiGyro gyro, SimulatedEncoder leftEncoder, SimulatedEncoder rightEncoder) {
        m_gyro = gyro;
        m_leftEncoder = leftEncoder;
        m_rightEncoder = rightEncoder;
        m_lastUpdateTime = Timer.getFPGATimestamp();
    }

    /**
     * Update the pose state based on the current speed controller outputs and time elapsed
     * @param leftMotor Output to left motor
     * @param rightMotor Output to right motor
     */
    public void updateSimulation(SpeedController leftMotor, SpeedController rightMotor) {
        /* Compute a delta and update m_lastUpdateTime. The delta makes the simulation
         * realtime-independent (i.e. if the robot runs slower or faster, we should
         * get close to the same answers).
         */
        double newTimestamp = Timer.getFPGATimestamp();
        double delta = newTimestamp - m_lastUpdateTime;
        m_lastUpdateTime = newTimestamp;

        /* Very simplified physics model used below:
         * We assume motion is completely described as 'linear' and 'turn' motion.
         * Turn is in the range ROTATIONS_PER_SECOND to -ROTATIONS_PER_SECOND, and is
         * determined by the difference between speed controller voltages / 2.
         * 
         * A real robot has several phenomena (momentum, friction, the chance to
         * skid) that we don't model here, but this is a good start.
         */ 
        // calculate so that right-motor-forward translates to counterclockwise (positive radians) motion
        double turn = (rightMotor.get() - leftMotor.get()) / 2;

        /* Now figure out how far the robot went this blip of time.
         * We'll pretend wheel motion scales completely linearly to speed controller
         * input. We multiply by delta to scale the whole computation by the fraction of
         * time that has passed, then add to previous encoder values.
         */
        m_leftEncoderValue += TICKS_PER_SECOND * leftMotor.get() * delta;
        m_rightEncoderValue += TICKS_PER_SECOND * rightMotor.get() * delta;

        /* Rotation is just scaling the max speed by how fast we're turning,
         * multiplying the result by delta to scale by the faction of time passed, and
         * adding it to the current rotation
         */
        m_rotation += ROTATIONS_PER_SECOND * turn * delta;

        /* Override encoders wtih simulated value */
        m_leftEncoder.set((int)m_leftEncoderValue);
        m_rightEncoder.set((int)m_rightEncoderValue);

        /* Override read gyro value with simulated value
         * Note: angle on gyro is flipped from right-hand rule; counterclockwise
         * chassis rotation makes angle more negative 
         */
       m_gyro.simulateAngleZ(m_rotation * -360.0);
    }
}
