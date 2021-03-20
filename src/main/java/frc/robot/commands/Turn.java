package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;

public class Turn extends CommandBase {
    private Hardware m_hardware;
    private DifferentialDrive m_drive;
    private double m_angle;
    // (0.006, 0.016, 0.0011
    // .02, 0, 0.001
    // 0.8: .01, .025, 0.004
    // 0.6: 0.04,0.0112,0.003

    private PIDController m_controller;

    private static final double MAX_SPEED = 0.5;
    private static final double POSITION_TOLERANCE = 1;
    private static final double CHANGE_TOLERANCE = 1;

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
        m_controller = new PIDController(0.03,0.0002,0.003);
        m_controller.setTolerance(POSITION_TOLERANCE, CHANGE_TOLERANCE);
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
        System.out.println("Target: " + m_controller.getSetpoint() + 
        " error: " + (m_controller.getSetpoint() - m_hardware.gyro().getAngleZ()) + 
        " value: " + output);
    }

    @Override public void end(boolean interrupt) {
        m_drive.stopMotor();
    }

    @Override public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
