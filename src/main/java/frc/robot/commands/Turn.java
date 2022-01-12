package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Hardware;

public class Turn extends CommandBase {
    private Hardware m_hardware;
    private DifferentialDrive m_drive;
    private double m_angle;

    private PIDController m_controller;
    private TrapezoidProfile m_profile;
    private double m_startTime;
    private double m_completionTime;

    // max speed in degrees / sec
    private static final double MAX_SPEED = 360.0;
    // max acceleration in degrees / sec^2
    private static final double MAX_ACCELERATION = 180.0;
    // When we reach the end of our trajectory, give a bit of time to let the PID control settle
    private static final double TIME_COMPLETION_SLOP = 0.25;

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
    }

    @Override public void initialize() {

        double currentGyroZ = m_hardware.gyro().getAngleZ();
        double targetGyroZ = currentGyroZ + m_angle;
        // P, I, D value experimentally determined with 90-degree turns
        m_controller = new PIDController(0.011, 
        0.0, 
        0.0);

        m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCELERATION),
            new TrapezoidProfile.State(targetGyroZ, 0),
            new TrapezoidProfile.State(currentGyroZ, 0));

        
        m_startTime = Timer.getFPGATimestamp();
        m_completionTime = m_startTime + m_profile.totalTime() + TIME_COMPLETION_SLOP;

        System.out.println("--- Going from " + currentGyroZ + " to " + targetGyroZ + " in " + m_profile.totalTime() + " seconds.");
    }

    @Override public void execute() {
        // Update the profile, then update the PID controller setpoint with where the profile says we should be
        m_controller.setSetpoint(m_profile.calculate(Timer.getFPGATimestamp() - m_startTime).position);
        // Now, what should the motors do to make the current position match the setpoint?
        double output = m_controller.calculate(m_hardware.gyro().getAngleZ());
        // positive rotations are clockwise, so should turn to the right
        m_drive.tankDrive(output, -output);
        System.out.println("Target: " + m_controller.getSetpoint() + 
        " error: " + (m_controller.getSetpoint() - m_hardware.gyro().getAngleZ()) + 
        " value: " + output);
    }

    @Override public void end(boolean interrupt) {
        m_drive.stopMotor();
    }

    @Override public boolean isFinished() {
        return Timer.getFPGATimestamp() > m_completionTime;
    }
}
