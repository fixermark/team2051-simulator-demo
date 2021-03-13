package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;

public class Turn extends CommandBase {
    private Hardware m_hardware;
    private DifferentialDrive m_drive;
    private double m_angle;
    private double m_targetAngle;
    private boolean m_turningRight;

    /**
     * Create the command
     * @param hardware The hardware to control
     * @param drive Differential drive to control on the robot
     * @param angle Total angle to turn, in degrees. Positive for clockwise, negative for counter-clockwise
     */
    public Turn(Hardware hardware, DifferentialDrive drive, double angle) {
        m_hardware = hardware;
        m_drive = drive;
        m_angle = angle;
        m_turningRight = angle > 0;
    }

    @Override public void initialize() {
        System.out.println("turn init");
        m_targetAngle = m_hardware.gyro().getAngleZ() + m_angle;
    }

    @Override public void execute() {
        System.out.println("Turning");
        double speed = 1.0;
        if (m_turningRight) {
            m_drive.tankDrive(speed, -speed);
        } else {
            m_drive.tankDrive(-speed, speed);
        }
    }

    @Override public void end(boolean interrupt) {
        m_drive.stopMotor();
    }

    @Override public boolean isFinished() {
        double angleZ = m_hardware.gyro().getAngleZ();
        if (m_turningRight) {
            return angleZ > m_targetAngle;
        } else {
            return angleZ < m_targetAngle;
        }
    }
}
