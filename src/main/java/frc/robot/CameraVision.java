package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

public class CameraVision {
    private final UsbCamera leftCamera;
    private final UsbCamera rightCamera;
    private final CvSink leftCvSink;
    private final CvSink rightCvSink;
    private final CvSource outputStream;
    private final AprilTagDetector detector;
    
    private static final double FX = 600; 
    private static final double FY = 600; 
    private static final double CX = 320; 
    private static final double CY = 240;
    private static final double TAG_SIZE = 0.165; 
    private static final double CAMERA_DISTANCE = 0.3; 

    public CameraVision(int leftCameraIndex, int rightCameraIndex) {
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

    public void detectTagsAndBalls() {
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
        outputStream.putFrame(leftFrame); 

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

        detectBall(leftFrame);
        detectBall(rightFrame);
    }

    private double[] estimatePose(AprilTagDetection detection) {
        double[] corners = detection.getCorners();
        double centerX = (corners[0] + corners[2] + corners[4] + corners[6]) / 4.0;
        double centerY = (corners[1] + corners[3] + corners[5] + corners[7]) / 4.0;

        double dx = (centerX - CX) / FX;
        double dy = (centerY - CY) / FY;

        double distance = TAG_SIZE / Math.sqrt(dx * dx + dy * dy);
        double angle = Math.toDegrees(Math.atan2(dy, dx));

        return new double[]{distance * 100, angle}; 
    }

    private double[] calculateRobotPosition(double[] leftPose, double[] rightPose) {
        double leftDistance = leftPose[0] / 100; 
        double rightDistance = rightPose[0] / 100; 

        double x = (leftDistance + rightDistance) / 2;
        double y = (rightDistance - leftDistance) / CAMERA_DISTANCE * TAG_SIZE;

        return new double[]{x * 100, y * 100}; 
    }

    private void detectBall(Mat frame) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar lowerCyan = new Scalar(80, 100, 100);
        Scalar upperCyan = new Scalar(100, 255, 255);

        Mat mask = new Mat();
        Core.inRange(hsv, lowerCyan, upperCyan, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(frame, contours, -1, new Scalar(0, 255, 0), 2);

        outputStream.putFrame(frame);

        System.out.println("Detected Balls: " + contours.size());
    }
}