package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;

public class LazyTurn extends CommandBase { 
    private Hardware m_hardware;
    private DifferentialDrive m_drive;
    private double m_degrees;
    private double m_targetRotation;

    /**
     * Create a command
     * @param hardware The hardware to build
     * @param drive Differential drive to control on the robot
     * @param degrees The total distance to go, in degrees
     */
    public LazyTurn(Hardware hardware, DifferentialDrive drive, double degrees) {
        m_hardware = hardware;
        m_drive = drive;
        m_degrees = degrees;
    }

    @Override public void initialize() {
        m_targetRotation = m_hardware.gyro().getAngleZ() - m_degrees;
    }

    @Override public void execute() {
        double leftSpeed = travelDone() ? 0 : 0.5;
        double rightSpeed = travelDone() ? 0 : 1;
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
