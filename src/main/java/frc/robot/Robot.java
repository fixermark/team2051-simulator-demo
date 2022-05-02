// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive;
import frc.robot.commands.LazyTurn;
import frc.robot.commands.Turn;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final Hardware m_hardware = new Hardware();
  private DifferentialDrive m_robotDrive;
  private final Joystick m_stick = new Joystick(0);
  private Field2d m_field;
  private SimpleSimulatedChassis m_chassis;
  private final PoseEstimator m_pose = new PoseEstimator();
  private DrivetrainMonitor m_drivetrainMonitor;
  private boolean m_realDetermined = false;
  private double m_startupDebounce = 0;

  public Robot() {
    m_robotDrive = new DifferentialDrive(m_hardware.leftMotor(), m_hardware.rightMotor());
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_startupDebounce = Timer.getFPGATimestamp();
    var commands = Shuffleboard.getTab("Auto control").getLayout("Commands", BuiltInLayouts.kList).withSize(1,3);

    commands.add("Drive", new Drive(m_hardware, m_robotDrive, 3));
    commands.add("Turn", new Turn(m_hardware, m_robotDrive, 72));
    commands.add("Lazy Turn", new LazyTurn(m_hardware, m_robotDrive, 72));

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

    m_drivetrainMonitor = new DrivetrainMonitor(m_pose);
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
      
      m_chassis = new SimpleSimulatedChassis(m_hardware);
    } 
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
    m_field.setRobotPose(m_pose.get());
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    double distance = 60;
    double turnDegrees = 90;
    //not sure about why it's not 90
    SequentialCommandGroup commands =  new SequentialCommandGroup(
      new Drive(m_hardware, m_robotDrive, inches(distance)),
      new Turn(m_hardware, m_robotDrive, -turnDegrees),
      new Drive(m_hardware, m_robotDrive, inches(distance)),
      new Turn(m_hardware, m_robotDrive, -turnDegrees),
      new Drive(m_hardware, m_robotDrive, inches(distance)),
      new Turn(m_hardware, m_robotDrive, -turnDegrees),
      new Drive(m_hardware, m_robotDrive, inches(distance)),
      new Turn(m_hardware, m_robotDrive, -turnDegrees));
      //somehow I changed something, distances seem to have been changed.
    CommandScheduler.getInstance().schedule(commands);
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
