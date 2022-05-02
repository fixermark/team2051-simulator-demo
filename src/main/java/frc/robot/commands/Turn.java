package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;

public class Turn extends CommandBase { private Hardware m_hardware;
    private DifferentialDrive m_drive;
    private double m_rotations;
    private double m_targetLeftEncoder;
    private double m_targetRightEncoder;
    private boolean m_drivingForward;

    private static final double MAX_SPEED =1;

    /**
     * Create a command
     * @param hardware The hardware to build
     * @param drive Differential drive to control on the robot
     * @param rotations The total distance to go, in encoder rotations
     */
    public Turn(Hardware hardware, DifferentialDrive drive, double rotations) {
        m_hardware = hardware;
        m_drive = drive;
        m_rotations = rotations;
        m_drivingForward = rotations > 0;
    }

    @Override public void initialize() {
        m_targetLeftEncoder = m_hardware.leftEncoderCount() + m_rotations;
        m_targetRightEncoder = m_hardware.rightEncoderCount() + m_rotations;
    }

    @Override public void execute() {
        var leftEncoder = m_hardware.leftEncoderCount();
        var rightEncoder = m_hardware.rightEncoderCount();

        double speed = m_drivingForward ? MAX_SPEED : -MAX_SPEED;

        double leftSpeed = travelDone(leftEncoder, m_targetLeftEncoder) ? 0 : -speed;
        double rightSpeed = travelDone(rightEncoder, m_targetRightEncoder) ? 0 : speed;
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    @Override public void end(boolean interrupt) {
        m_drive.stopMotor();
    }

    /**
     * Signals the Drive Command is completed.
     */
    @Override public boolean isFinished() {
        return 
        travelDone(m_hardware.leftEncoderCount(), m_targetLeftEncoder) &&
        travelDone(m_hardware.rightEncoderCount(), m_targetRightEncoder);
    }

    private boolean travelDone(double currentEncoder, double targetEncoder) {
        if (m_drivingForward) {
            return currentEncoder > targetEncoder;
        } else {
            return currentEncoder < targetEncoder;
        }
    }
}
