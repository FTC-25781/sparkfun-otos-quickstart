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
 //
    @Override
    public void runOpMode() {
        // Initialize hardware
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // This is the servo controlling the claw
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Reset the claw servo to 0.0 at the start of the OpMode
        clawServo.setPosition(0.0);
        sleep(500);  // Allow half a second for the servo to reach the position

        // Set the OpenCV pipeline
        webcam.setPipeline(new BlockDetectionPipeline());

        // Open the camera asynchronously
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
            telemetry.update();
            sleep(50);
        }
    }

    class BlockDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Convert to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define HSV range for blue
            Scalar lowerBlue = new Scalar(105, 170, 50);
            Scalar upperBlue = new Scalar(130, 255, 255);

            // Define HSV range for red
            Scalar lowerRed1 = new Scalar(0, 170, 50);    // Lower bound of red (for values near 0 in hue)
            Scalar upperRed1 = new Scalar(10, 255, 255);  // Upper bound of red (for values near 0 in hue)
            Scalar lowerRed2 = new Scalar(170, 170, 50);  // Lower bound of red (for values near 180 in hue)
            Scalar upperRed2 = new Scalar(180, 255, 255); // Upper bound of red (for values near 180 in hue)

            // Define HSV range for yellow
            Scalar lowerYellow = new Scalar(20, 170, 50);
            Scalar upperYellow = new Scalar(40, 255, 255);

            // Mask blue areas
            Mat blueMask = new Mat();
            Core.inRange(hsv, lowerBlue, upperBlue, blueMask);

            // Mask red areas
            Mat redMask1 = new Mat();
            Mat redMask2 = new Mat();
            Core.inRange(hsv, lowerRed1, upperRed1, redMask1);
            Core.inRange(hsv, lowerRed2, upperRed2, redMask2);
            Mat redMask = new Mat();
            Core.add(redMask1, redMask2, redMask);

            // Mask yellow areas
            Mat yellowMask = new Mat();
            Core.inRange(hsv, lowerYellow, upperYellow, yellowMask);

            // Combine all masks into one
            Mat combinedMask = new Mat();
            Core.bitwise_or(blueMask, redMask, combinedMask);
            Core.bitwise_or(combinedMask, yellowMask, combinedMask);

            // Blur the mask to reduce noise
            Imgproc.GaussianBlur(combinedMask, combinedMask, new Size(5, 5), 0);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Process contours
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 500) {
                    // Fit a rotated rectangle around the contour
                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    RotatedRect rect = Imgproc.minAreaRect(contour2f);

                    // Draw the bounding box
                    Point[] points = new Point[4];
                    rect.points(points);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(input, points[i], points[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                    }

                    // Get the angle of rotation
                    double angle = rect.angle;
                    if (rect.size.width < rect.size.height) {
                        angle = -angle;
                    } else {
                        angle = 90 - angle;
                    }

                    // Convert angle to a value between 0 and 180
                    angle = Math.abs(angle) % 180;

                    // Map angle to servo position (0.0 to 1.0)
                    double servoPosition = angle / 180.0;
                    clawServo.setPosition(servoPosition);

                    telemetry.addData("Block angle (degrees)", angle);
                    telemetry.addData("Servo position", servoPosition);
                }
            }

            return input;
        }
    }
}
