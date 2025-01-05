package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagDetection;
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
        aprilTagDetector = new AprilTagVision(0); // Initialize with camera index 0
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Detect AprilTags and print their IDs
        List<AprilTagDetection> detections = aprilTagDetector.detectTags();
        for (AprilTagDetection detection : detections) {
            System.out.println("Detected AprilTag ID: " + detection.getId());
        }
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
