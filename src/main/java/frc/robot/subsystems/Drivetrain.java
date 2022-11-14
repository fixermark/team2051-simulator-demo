package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.SimpleSimulatedChassis;

public class Drivetrain extends SubsystemBase {
    private Hardware m_hardware;
    private DifferentialDrive m_drive;
    private DifferentialDriveOdometry m_odometry;

    private double m_leftWheelSpeed = 0;
    private double m_rightWheelSpeed = 0;

    private int m_lastLeftEncoder = 0;
    private int m_lastRightEncoder = 0;
    private double m_lastTimestamp = 0;

    public static final double METERS_PER_TICK = SimpleSimulatedChassis.WHEEL_CIRCUMFERENCE_METERS /
    (double)SimpleSimulatedChassis.ENCODER_TICKS_PER_REVOLUTION;

    // Some test constants
    public static final double MOTOR_KS_VOLTS = 0.5;
    public static final double MOTOR_KV_VOLT_SEC_PER_M = 1.98;
    public static final double MOTOR_KA_VOLT_SEC2_PER_M = 0.2;

    public static final double MAX_SPEED_M_S = 0.1;
    public static final double MAX_ACCEL_M_S2 = 8;
    public static final double TRACK_WIDTH_M = 0.7;

    public static final double DRIVE_VEL_P = 8.5;

    public static final double RAMSETE_B = 2;  // was 2
    public static final double RAMSETE_ZETA = .7;  // was 0.7

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_M);

    public Drivetrain(Hardware hardware) {
        m_hardware = hardware;
        m_drive = new DifferentialDrive(m_hardware.leftMotor(), m_hardware.rightMotor());
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_hardware.gyro().getAngleZ()));
        m_lastLeftEncoder = m_hardware.leftEncoderCount();
        m_lastRightEncoder = m_hardware.rightEncoderCount();
    }

    @Override
    public void periodic() {
        var curTime = Timer.getFPGATimestamp();
        var deltaT = curTime - m_lastTimestamp;
        var leftEncoder = m_hardware.leftEncoderCount();
        var rightEncoder = m_hardware.rightEncoderCount();

        m_leftWheelSpeed = wheelVelocity(leftEncoder, m_lastLeftEncoder, deltaT);
        m_rightWheelSpeed = wheelVelocity(rightEncoder, m_lastRightEncoder, deltaT);

        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);

        m_odometry.update(
            Rotation2d.fromDegrees(-m_hardware.gyro().getAngleZ()),
            wheelDistance(leftEncoder),
            wheelDistance(rightEncoder));

        m_lastTimestamp = curTime;
        m_lastLeftEncoder = leftEncoder;
        m_lastRightEncoder = rightEncoder;

        SmartDashboard.putNumber("leftMotorPower", m_hardware.leftMotor().get());
        SmartDashboard.putNumber("rightMotorPower",  m_hardware.rightMotor().get());
    }

    public DifferentialDrive drive() {
        return m_drive;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    private double wheelVelocity(double count, double lastCount, double deltaT) {
        return (count - lastCount) * METERS_PER_TICK;
    }

    private double wheelDistance(double count) {
        return count * METERS_PER_TICK;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftWheelSpeed, m_rightWheelSpeed);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_drive.tankDrive(leftVolts / 12.0, rightVolts / 12.0);
    }

    public TrajectoryConfig getTrajectoryConfig() {
        var config = new TrajectoryConfig(MAX_SPEED_M_S, MAX_ACCEL_M_S2);
        config.setKinematics(DRIVE_KINEMATICS);
        return config;
    }

    public CommandBase followTrajectoryCommand(Trajectory trajectory) {
       return new RamseteCommand(
            trajectory, 
            this::getPose,
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            new SimpleMotorFeedforward(MOTOR_KS_VOLTS, MOTOR_KV_VOLT_SEC_PER_M, MOTOR_KA_VOLT_SEC2_PER_M),
            DRIVE_KINEMATICS,
            this::getWheelSpeeds,
            new PIDController(DRIVE_VEL_P, 0,0),
            new PIDController(DRIVE_VEL_P, 0, 0),
            this::tankDriveVolts, this).andThen(() -> tankDriveVolts(0,0));
    
    }
}
