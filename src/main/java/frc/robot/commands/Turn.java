package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;

public class Turn extends CommandBase { private Hardware m_hardware;
    private DifferentialDrive m_drive;
    private double m_degrees;
    private double m_targetRotation;
    private boolean m_drivingForward;

    private static final double MAX_SPEED =1;

    /**
     * Create a command
     * @param hardware The hardware to build
     * @param drive Differential drive to control on the robot
     * @param degrees The total distance to go, in degrees
     */
    public Turn(Hardware hardware, DifferentialDrive drive, double degrees) {
        m_hardware = hardware;
        m_drive = drive;
        m_degrees = degrees;
        m_drivingForward = degrees > 0;
    }

    @Override public void initialize() {
        m_targetRotation = m_hardware.gyro().getAngleZ() - m_degrees;
    }

    @Override public void execute() {
        double speed = m_drivingForward ? MAX_SPEED : -MAX_SPEED;

        double leftSpeed = travelDone() ? 0 : -speed;
        double rightSpeed = travelDone() ? 0 : speed;
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    @Override public void end(boolean interrupt) {
        m_drive.tankDrive(0,0);
    }

    /**
     * Signals the Drive Command is completed.
     */
    @Override public boolean isFinished() {
        return travelDone();
    }

    private boolean travelDone() {
        return m_hardware.gyro().getAngleZ() < m_targetRotation;
    }
}
