package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "BlockDetectionOpMode", group = "Linear Opmode")
public class BlockDetectionOpMode extends LinearOpMode {
    private OpenCvCamera webcam;
    private Servo clawServo;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setPosition(0.0);  // Ensure the claw is at the starting position
        sleep(500);

        BlockDetectionPipeline blkp =   new BlockDetectionPipeline();
        webcam.setPipeline(blkp);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Could not open camera");
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) blkp.hasDetectedBlock = false;
            telemetry.update();
            sleep(50);
        }
    }

    class BlockDetectionPipeline extends OpenCvPipeline {
        private static final double TOLERANCE_DEGREES = 5.0;
        private static final double SMOOTHING_FACTOR = 0.1;
        private double lastServoPosition = 0.0;
        public boolean hasDetectedBlock = false;

        @Override
        public Mat processFrame(Mat input) {

            if (hasDetectedBlock) return input;

            // Ensure the servo starts at the position 0
            clawServo.setPosition(0);
            sleep(2000);

            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define HSV ranges for blue, red, and yellow
            Scalar lowerBlue = new Scalar(105, 170, 50);
            Scalar upperBlue = new Scalar(130, 255, 255);

            Scalar lowerRed1 = new Scalar(0, 170, 50);
            Scalar upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 170, 50);
            Scalar upperRed2 = new Scalar(180, 255, 255);

            Scalar lowerYellow = new Scalar(20, 170, 50);
            Scalar upperYellow = new Scalar(40, 255, 255);

            // Masks for blue, red, and yellow
            Mat blueMask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, blueMask);

            Mat redMask1 = new Mat();
            Mat redMask2 = new Mat();
            Core.inRange(hsv, lowerRed1, upperRed1, redMask1);
            Core.inRange(hsv, lowerRed2, upperRed2, redMask2);
            Mat redMask = new Mat();
            Core.add(redMask1, redMask2, redMask);

            Mat yellowMask = new Mat();
            Core.inRange(hsv, lowerYellow, upperYellow, yellowMask);

            // Combine all masks
            Mat combinedMask = new Mat();
            Core.bitwise_or(blueMask, redMask, combinedMask);
            Core.bitwise_or(combinedMask, yellowMask, combinedMask);

            // Blur to reduce noise
            Imgproc.GaussianBlur(combinedMask, combinedMask, new Size(5, 5), 0);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Variables to track the largest block found
            double maxArea = 0;
            RotatedRect bestRect = null;

            // Process contours
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 500 && area > maxArea) { // Only consider larger blocks
                    maxArea = area;
                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    bestRect = Imgproc.minAreaRect(contour2f);
                }
            }

            // If a block is detected, lock onto it
            if (bestRect != null) {
                // Draw bounding box for the detected block
                Point[] points = new Point[4];
                bestRect.points(points);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, points[i], points[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                // Calculate block angle
                double angle = bestRect.angle;

                // Adjust the angle based on the camera's perspective
                if (bestRect.size.width < bestRect.size.height) {
                    angle = -angle; // Flip for vertical block alignment
                } else {
                    angle = 90 - angle; // Adjust for horizontal alignment
                }

                // Flip the angle to align left to right (camera orientation)
                angle = Math.abs(angle) % 180;

                // Map the angle to the servo's range (0.0 to 0.5)
                hasDetectedBlock = true;
                double servoTargetPosition = (angle / 90.0) * 0.5;

                // Gradually move the servo towards the target position (0.0)
                if (Math.abs(lastServoPosition - 0.0) > TOLERANCE_DEGREES) {
                    lastServoPosition = lastServoPosition - SMOOTHING_FACTOR * (lastServoPosition - 0.0);
                    clawServo.setPosition(lastServoPosition);
                } else {
                    clawServo.setPosition(0.0); // Lock servo at position 0
                }

                // For debugging, output the corresponding angle for the servo position
                double mappedAngle = servoTargetPosition * 90; // Map the servo position back to degrees

                telemetry.addData("Detected Block Angle (degrees)", angle);
                telemetry.addData("Mapped Angle (degrees)", mappedAngle);
                telemetry.addData("Servo Target Position", servoTargetPosition);
            }
            return input;
        }
    }
}
