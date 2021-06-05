package frc.robot.test;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.PoseEstimator;
import frc.robot.Hardware;
import frc.robot.SimulatedEncoder;
import frc.robot.sensors.RomiGyro;

public class PoseEstimatorTest {
    private PoseEstimator estimator;
    private SimulatedEncoder leftEncoder;
    private SimulatedEncoder rightEncoder;
    private RomiGyro gyro;
    
    @Before
    public void setUp() {

        estimator = new PoseEstimator();

        leftEncoder = new SimulatedEncoder();
        rightEncoder = new SimulatedEncoder();
        gyro = new RomiGyro();

        estimator.initPose(leftEncoder, rightEncoder);
    }

    @Test
    public void testDriveStraight() {
        leftEncoder.set(1000);
        rightEncoder.set(1000);

        estimator.updatePose(leftEncoder, rightEncoder, gyro);

        double expectedDistance = PoseEstimator.DISTANCE_PER_TICK * 1000;

        Assert.assertEquals(
            new Pose2d(expectedDistance, 0, new Rotation2d(0)), 
            estimator.getPose()
            );
    }

    @Test
    public void testTurn() {

    }
}
