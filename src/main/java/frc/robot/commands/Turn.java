package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;

public class Turn extends CommandBase {
    private Hardware m_hardware;
    private DifferentialDrive m_drive;
    private double m_angle;
    private PIDController m_controller = new PIDController(0.1, 0.001, -.001);

    private static final double MAX_SPEED = 1.0;
    private static final double TOLERANCE = 0.5;

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
        m_controller.setTolerance(TOLERANCE);
    }

    @Override public void initialize() {
        m_controller.reset();
        m_controller.setSetpoint(m_hardware.gyro().getAngleZ() + m_angle);
    }

    @Override public void execute() {
        double output = m_controller.calculate(m_hardware.gyro().getAngleZ());
        double clampedOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, output));
        // positive rotations are clockwise, so should turn to the right
        m_drive.tankDrive(clampedOutput, -clampedOutput);
    }

    @Override public void end(boolean interrupt) {
        m_drive.stopMotor();
    }

    @Override public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
