package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReportTrajectory extends CommandBase {
    Trajectory m_trajectory;
    Timer m_timer = new Timer();

    public ReportTrajectory(Trajectory trajectory) {
        m_trajectory = trajectory;
    }


    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        var pose = m_trajectory.sample(m_timer.get());
        SmartDashboard.putNumber("setpointX", pose.poseMeters.getX());
        SmartDashboard.putNumber("setpointY", pose.poseMeters.getY());
        SmartDashboard.putNumber("setpointVelocity", pose.velocityMetersPerSecond);
        SmartDashboard.putNumber("setpointAcceleration", pose.accelerationMetersPerSecondSq);
    }

    @Override
    public void end(boolean override) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }  
}
