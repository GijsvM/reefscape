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
    private final UsbCamera camera;
    private final CvSink cvSink;
    private final CvSource outputStream;
    private final AprilTagDetector detector;

    // Camera intrinsic parameters (example values, you need to calibrate your camera to get these)
    private static final double FX = 600; // Focal length in pixels
    private static final double FY = 600; // Focal length in pixels
    private static final double CX = 320; // Principal point x-coordinate in pixels
    private static final double CY = 240; // Principal point y-coordinate in pixels
    private static final double TAG_SIZE = 0.165; // Size of the AprilTag in meters (example: 16.5 cm)

    public AprilTagVision(int cameraIndex) {
        camera = CameraServer.startAutomaticCapture(cameraIndex);
        camera.setResolution(640, 480);

        cvSink = CameraServer.getVideo();
        outputStream = CameraServer.putVideo("Detected", 640, 480);

        detector = new AprilTagDetector();
        detector.addFamily("tag36h11");
    }

    public List<AprilTagDetection> detectTags() {
        Mat frame = new Mat();
        if (cvSink.grabFrame(frame) == 0) {
            return new ArrayList<>();
        }

        Mat gray = new Mat();
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

        AprilTagDetection[] detectionsArray = detector.detect(gray);
        List<AprilTagDetection> detections = new ArrayList<>(List.of(detectionsArray));
        outputStream.putFrame(frame);

        for (AprilTagDetection detection : detections) {
            double[] pose = estimatePose(detection);
            System.out.println("Detected AprilTag ID: " + detection.getId());
            System.out.println("Distance: " + pose[0] + " cm");
            System.out.println("Angle: " + pose[1] + " degrees");
        }

        return detections;
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
}