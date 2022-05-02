package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple monitor for simulated robot operation: tracks robot's speed, acceleration, and position every frame
 * and outputs them to Shuffleboard
 */
public class DrivetrainMonitor extends SubsystemBase {
    private static final Translation2d ZERO_TRANSLATION = new Translation2d(0,0);

    // Supplier of the monitored robot pose data
    private Supplier<Pose2d> m_poseSource;

    private Pose2d m_firstPose;
    private Pose2d m_lastPose;

    private double m_lastLinearVelocity;
    private double m_lastAngularVelocity;

    // NetworkTableEntries monitoring the characteristics of the chassis motion
    private NetworkTableEntry m_xDelta;
    private NetworkTableEntry m_yDelta;
    private NetworkTableEntry m_delta;
    private NetworkTableEntry m_heading;

    private NetworkTableEntry m_linearVelocity;
    private NetworkTableEntry m_linearAcceleration;
    private NetworkTableEntry m_angularVelocity;
    private NetworkTableEntry m_angularAcceleration;
    
    /**
     * Constructor
     * @param poseSource A source to fetch poses in periodic loop
     */
    public DrivetrainMonitor(Supplier<Pose2d> poseSource) {
        m_poseSource = poseSource;

        var positionsTab = Shuffleboard.getTab("simulated chassis positions");
        var deltasTab = Shuffleboard.getTab("simulated chassis velocities and accelerations");

        m_xDelta = positionsTab.add("Change in X position (meters)", 0).withPosition(0,0).withSize(3,3).withWidget(BuiltInWidgets.kGraph).getEntry();
        m_yDelta = positionsTab.add("Change in Y position (meters)", 0).withPosition(3,0).withSize(3,3).withWidget(BuiltInWidgets.kGraph).getEntry();
        m_delta = positionsTab.add("Change in position (meters)", 0).withPosition(0,3).withSize(3,3).withWidget(BuiltInWidgets.kGraph).getEntry();
        m_heading = positionsTab.add("Heading (degrees)", 0).withPosition(3,3).withSize(3,3).withWidget(BuiltInWidgets.kGraph).getEntry();

        m_linearVelocity = deltasTab.add("Linear velocity (m per sec)", 0).withPosition(0,0).withSize(3,3).withWidget(BuiltInWidgets.kGraph).getEntry();
        m_linearAcceleration = deltasTab.add("Linear acceleration (m per sec^2)", 0).withPosition(3,0).withSize(3,3).withWidget(BuiltInWidgets.kGraph).getEntry();

        m_angularVelocity = deltasTab.add("Angular velocity (degrees per sec)", 0).withPosition(0,3).withSize(3,3).withWidget(BuiltInWidgets.kGraph).getEntry();
        m_angularAcceleration = deltasTab.add("Angular acceleration (degrees per sec^2", 0).withPosition(3,3).withSize(3,3).withWidget(BuiltInWidgets.kGraph).getEntry();
    }

    @Override
    public void periodic() {
        if (m_firstPose == null) {
            init();
            return;
        }


        var pose = m_poseSource.get();
        var position = pose.getTranslation();
        var heading = pose.getRotation();

        updateLinearMetrics(position);
        updateAngularMetrics(heading);

        m_lastPose = pose;
    }

    /**
     * Update all linear metrics (display and saved state)
     * 
     * @param position Position of the robot
     */
    private void updateLinearMetrics(Translation2d position) {
        var deltaFromStart = position.minus(m_firstPose.getTranslation());
        var delta = deltaFromStart.getDistance(ZERO_TRANSLATION);

        var deltaFromLastStep = position.minus(m_lastPose.getTranslation());
        var velocity = deltaFromLastStep.getDistance(ZERO_TRANSLATION) * TimedRobot.kDefaultPeriod;

        var acceleration = velocity - m_lastLinearVelocity;

        // store linear state
        m_lastLinearVelocity = velocity;

        // update metrics
        m_xDelta.setDouble(deltaFromStart.getX());
        m_yDelta.setDouble(deltaFromStart.getY());
        m_delta.setDouble(delta);

        m_linearVelocity.setDouble(velocity);
        m_linearAcceleration.setDouble(acceleration);
    }

    /**
     * Update all angular metrics (display and saved state)
     * @param heading Heading of the robot
     */
    private void updateAngularMetrics(Rotation2d heading) {
        var headingDegrees = heading.getDegrees();

        var velocity = (headingDegrees - m_lastPose.getRotation().getDegrees()) * TimedRobot.kDefaultPeriod;
        var acceleration = velocity - m_lastAngularVelocity;

        // store angular state
        m_lastAngularVelocity = velocity;

        // update metrics
        m_heading.setDouble(headingDegrees);
        m_angularVelocity.setDouble(velocity);
        m_angularAcceleration.setDouble(acceleration);
    }

    private void init() {
        m_firstPose = m_poseSource.get();
        m_lastPose = m_firstPose;
    }
}
