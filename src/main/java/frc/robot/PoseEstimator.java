package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Estimates robot's x, y, and rotational pose on the field from encoder and gyro values
 */
public class PoseEstimator {
    private int m_lastLeftEncoderValue;
    private int m_lastRightEncoderValue;
    private Pose2d m_pose = new Pose2d();
 
    /**
     * Number of encoder counts per full wheel revolution
     */
    private static final double ENCODER_TICKS_PER_REVOLUTION = 1440.0;

    /**
     * Circumference of wheel in meters
     */
    private static final double WHEEL_CIRCUMFERENCE_METERS = 0.07 * Math.PI;

    /**
     * Distance per tick
     */
    private static final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE_METERS / ENCODER_TICKS_PER_REVOLUTION;

    /**
     * Initialize the pose estimator
     * @param hardware Hardware on which to estimate pose
     */
    public void initPose(Hardware hardware) {
        m_lastLeftEncoderValue = hardware.leftEncoderCount();
        m_lastRightEncoderValue = hardware.rightEncoderCount();
    }

    /**
     * Update the pose state based on the current speed controller outputs and time elapsed
     * @param leftMotor Output to left motor
     * @param rightMotor Output to right motor
     */
    public void updatePose(Hardware hardware) {
        /* Very simplified physics model used below:
         * We assume forward motion is just average of motion of both wheels.
         */ 
        int leftCount = hardware.leftEncoderCount() - m_lastLeftEncoderValue;
        int rightCount = hardware.rightEncoderCount() - m_lastRightEncoderValue;

        m_lastLeftEncoderValue = hardware.leftEncoderCount();
        m_lastRightEncoderValue = hardware.rightEncoderCount();
        
        double linear = ((double)(leftCount + rightCount)) / 2.0;
        // note flip in angle: gyro follows left-hand rule
        double rotation = -hardware.gyro().getAngleZ() * Math.PI / 180.0;

        /* Now figure out how far the robot went this blip of time.
         * Linear is the distance, but the x- and y-axis distance is figured out by
         * trigonometry on the current heading. Finally, we multiply
         * by delta to scale the whole computation by the fraction of
         * time that has passed and add it to the current position.
         */
        double newX = DISTANCE_PER_TICK * linear * Math.cos(rotation) + m_pose.getX();
        double newY = DISTANCE_PER_TICK * linear * Math.sin(rotation) + m_pose.getY();

        /* Create a new pose with thesee updated values 
         * Note: angle is flipped from right-hand rule; gets more negative as
         * chassis rotates counter-clockwise
         */
        m_pose = new Pose2d(
            new Translation2d(newX, newY), 
            Rotation2d.fromDegrees(-hardware.gyro().getAngleZ()));
    }

    /**
     * Get most-recently-calculated pose value
     * @return pose value
     */
    public Pose2d getPose() {
        return m_pose;
    }
}
