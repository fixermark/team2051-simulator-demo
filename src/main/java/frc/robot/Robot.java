// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.Turn;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final Hardware m_hardware = new Hardware();
  private Drivetrain m_drivetrain;
  private final Joystick m_stick = new Joystick(0);
  private Field2d m_field;
  private SimpleSimulatedChassis m_chassis;
  private PoseEstimator m_pose = new PoseEstimator();
  private boolean m_realDetermined = false;
  private double m_startupDebounce = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_startupDebounce = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Initialization done when we are simulating
   */
  @Override
  public void simulationInit() {
    m_field = new Field2d();

    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Initializes the robot with information on whether it's simulated.
   * 
   * This is run from simulationPeriodic because the robot reads DIO to determine
   * if it is real or simulated, and DIO isn't populated in robotInit or simulationInit
   */
  private void initWithSensors() {
    if(m_hardware.isReal()) {
      System.out.println("Robot believes it is talking to real hardware");
    } else {
      System.out.println("Robot believes it is simulated");
      SimulatedEncoder left = new SimulatedEncoder();
      SimulatedEncoder right = new SimulatedEncoder();
      m_hardware.simulateEncoders(left, right);
      m_chassis = new SimpleSimulatedChassis(m_hardware.gyro(), left, right);
    } 
    m_drivetrain = new Drivetrain(m_hardware);
  }

  /** For convenience, inches-to-meters conversion */
  private double inches(double inches) {
    return inches * 0.0254 ;  
  }

  /**
   * Update the simulation
   */
  @Override
  public void simulationPeriodic() {
    // Give the "real boy" pin a chance to be read
    if (Timer.getFPGATimestamp() - m_startupDebounce < 0.25) {
      return;
    }
    if (!m_realDetermined) {
      initWithSensors();
      m_pose.initPose(m_hardware);
      m_realDetermined = true;
    }

    if (m_chassis != null) {
      m_chassis.updateSimulation(m_hardware.leftMotor(), m_hardware.rightMotor());
    }
    /*
     * Simulate the robot's position based on the current motion
     */
    m_pose.updatePose(m_hardware);
    m_field.setRobotPose(m_pose.getPose());
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    var trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(new Translation2d(1,1), new Translation2d(2, 3)), 
      new Pose2d(4, 3, new Rotation2d(0)), 
      m_drivetrain.getTrajectoryConfig());
    var command = m_drivetrain.followTrajectoryCommand(trajectory);
    CommandScheduler.getInstance().schedule(command);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_drivetrain.drive().arcadeDrive(-m_stick.getY(), m_stick.getX());
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
