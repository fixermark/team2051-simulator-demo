package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * A simulation of the robot's chassis, converting input drive voltages to
 * changes to a robot pose
 */
public class SimpleSimulatedChassis {
    /**
     * Speed at which robot drives forward in meters per second at max velocity
     */
    private static final double FORWARD_SPEED_MS = 2.0;

    /** 
     * Speed at which robot rotates in radians per second at max rotation
     */
    private static final double ROTATION_SPEED_RADS = Math.PI;


    private Pose2d m_pose = new Pose2d();
    private double m_lastUpdateTime;

    public SimpleSimulatedChassis () {
        m_lastUpdateTime = Timer.getFPGATimestamp();
    }

    /**
     * Update the pose state based on the current speed controller outputs and time elapsed
     * @param leftMotor Output to left motor
     * @param rightMotor Output to right motor
     */
    public void updatePose(SpeedController leftMotor, SpeedController rightMotor) {
        /* Compute a delta and update m_lastUpdateTime. The delta makes the simulation
         * realtime-independent (i.e. if the robot runs slower or faster, we should
         * get close to the same answers).
         */
        double newTimestamp = Timer.getFPGATimestamp();
        double delta = newTimestamp - m_lastUpdateTime;
        m_lastUpdateTime = newTimestamp;

        /* Very simplified physics model used below:
         * We assume motion is completely described as 'linear' and 'turn' motion.
         * Linear is just in the range FORWARD_SPEED_MS to -FORWARD_SPEED_MS, and
         * determined by the average of the speed controller voltages. Turn
         * is in the range ROTATION_SPEED_RADS to -ROTATION_SPEED_RADS, and is
         * determined by the difference between speed controller voltages / 2.
         * 
         * A real robot has several phenomena (momentum, friction, the chance to
         * skid) that we don't model here, but this is a good start.
         */ 
        double linear = leftMotor.get() + rightMotor.get() / 2;
        // calculate so that right-motor-forward translates to counterclockwise (positive radians) motion
        double turn = (rightMotor.get() - leftMotor.get()) / 2;

        /* Now figure out how far the robot went this blip of time.
         * Linear is the speed, but the direction is figured out by
         * trigonometry on the current heading. Finally, we multiply
         * by delta to scale the whole computation by the fraction of
         * time that has passed and add it to the current position.
         */
        double newX = FORWARD_SPEED_MS * linear * m_pose.getRotation().getCos() * delta + m_pose.getX();
        double newY = FORWARD_SPEED_MS * linear * m_pose.getRotation().getSin() * delta + m_pose.getY();

        /* Rotation is just scaling the max speed by how fast we're turning,
         * multiplying the result by delta to scale by the faction of time passed, and
         * adding it to the current rotation...
         */
        double newRotation = ROTATION_SPEED_RADS * turn * delta + m_pose.getRotation().getRadians();
        /* ... but since rotation is in the range 0 to 2*PI radians, we need to 
         * remove chunks of 2 * PI if we're over the limit or add chunks of 2 * PI if 
         * we're under.
         */
        while (newRotation > 2 * Math.PI) {
            newRotation = newRotation - 2 * Math.PI;
        }
        while (newRotation <= 0) {
            newRotation = newRotation + 2 * Math.PI;
        }

        /* Create a new pose with thesee updated values */
        m_pose = new Pose2d(
            new Translation2d(newX, newY), 
            new Rotation2d(newRotation));
    }

    /**
     * Get our most recent pose
     * @return The pose
     */
    public Pose2d getPose() {
        return m_pose;
    }
}
