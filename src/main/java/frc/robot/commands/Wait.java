package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    private double m_secondsToAdd;
    private double m_targetTime;

    public Wait(double seconds) {
        m_secondsToAdd = seconds;
    }

    @Override public void initialize() {
        m_targetTime = Timer.getFPGATimestamp() + m_secondsToAdd;
    }

    @Override public boolean isFinished() {
        return m_targetTime < Timer.getFPGATimestamp();
    }

}
