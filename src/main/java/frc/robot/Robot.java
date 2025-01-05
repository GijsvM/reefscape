package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private AprilTagVision aprilTagDetector;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        aprilTagDetector = new AprilTagVision(0, 1); // Initialize with left and right camera indices
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Detect AprilTags and print their IDs and positions
        aprilTagDetector.detectTags();
    }

    @Override
    public void disabledInit() {
        // No need to release resources as WPILib handles it
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}