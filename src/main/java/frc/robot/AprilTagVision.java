package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

public class AprilTagVision {
    private final UsbCamera leftCamera;
    private final UsbCamera rightCamera;
    private final CvSink leftCvSink;
    private final CvSink rightCvSink;
    private final CvSource outputStream;
    private final AprilTagDetector detector;

    // Camera intrinsic parameters (example values, you need to calibrate your camera to get these)
    private static final double FX = 600; // Focal length in pixels
    private static final double FY = 600; // Focal length in pixels
    private static final double CX = 320; // Principal point x-coordinate in pixels
    private static final double CY = 240; // Principal point y-coordinate in pixels
    private static final double TAG_SIZE = 0.165; // Size of the AprilTag in meters (example: 16.5 cm)
    private static final double CAMERA_DISTANCE = 0.3; // Distance between the two cameras in meters

    public AprilTagVision(int leftCameraIndex, int rightCameraIndex) {
        leftCamera = CameraServer.startAutomaticCapture(leftCameraIndex);
        leftCamera.setResolution(640, 480);

        rightCamera = CameraServer.startAutomaticCapture(rightCameraIndex);
        rightCamera.setResolution(640, 480);

        leftCvSink = CameraServer.getVideo(leftCamera);
        rightCvSink = CameraServer.getVideo(rightCamera);
        outputStream = CameraServer.putVideo("Detected", 640, 480);

        detector = new AprilTagDetector();
        detector.addFamily("tag36h11");
    }

    public void detectTags() {
        Mat leftFrame = new Mat();
        Mat rightFrame = new Mat();

        if (leftCvSink.grabFrame(leftFrame) == 0 || rightCvSink.grabFrame(rightFrame) == 0) {
            return;
        }

        Mat leftGray = new Mat();
        Mat rightGray = new Mat();
        Imgproc.cvtColor(leftFrame, leftGray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(rightFrame, rightGray, Imgproc.COLOR_BGR2GRAY);

        AprilTagDetection[] leftDetectionsArray = detector.detect(leftGray);
        AprilTagDetection[] rightDetectionsArray = detector.detect(rightGray);
        List<AprilTagDetection> leftDetections = new ArrayList<>(List.of(leftDetectionsArray));
        List<AprilTagDetection> rightDetections = new ArrayList<>(List.of(rightDetectionsArray));
        outputStream.putFrame(leftFrame); // Display left frame for simplicity

        for (AprilTagDetection leftDetection : leftDetections) {
            for (AprilTagDetection rightDetection : rightDetections) {
                if (leftDetection.getId() == rightDetection.getId()) {
                    double[] leftPose = estimatePose(leftDetection);
                    double[] rightPose = estimatePose(rightDetection);
                    double[] robotPosition = calculateRobotPosition(leftPose, rightPose);

                    System.out.println("Detected AprilTag ID: " + leftDetection.getId());
                    System.out.println("Left Camera Distance: " + leftPose[0] + " cm");
                    System.out.println("Right Camera Distance: " + rightPose[0] + " cm");
                    System.out.println("Robot Position: X = " + robotPosition[0] + " cm, Y = " + robotPosition[1] + " cm");
                }
            }
        }
    }

    private double[] estimatePose(AprilTagDetection detection) {
        double[] corners = detection.getCorners();
        double centerX = (corners[0] + corners[2] + corners[4] + corners[6]) / 4.0;
        double centerY = (corners[1] + corners[3] + corners[5] + corners[7]) / 4.0;

        double dx = (centerX - CX) / FX;
        double dy = (centerY - CY) / FY;

        double distance = TAG_SIZE / Math.sqrt(dx * dx + dy * dy);
        double angle = Math.toDegrees(Math.atan2(dy, dx));

        return new double[]{distance * 100, angle}; // Convert distance to centimeters
    }

    private double[] calculateRobotPosition(double[] leftPose, double[] rightPose) {
        double leftDistance = leftPose[0] / 100; // Convert to meters
        double rightDistance = rightPose[0] / 100; // Convert to meters

        double x = (leftDistance + rightDistance) / 2;
        double y = (rightDistance - leftDistance) / CAMERA_DISTANCE * TAG_SIZE;

        return new double[]{x * 100, y * 100}; // Convert to centimeters
    }
}