package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Teleop", group = "Teleop")
public class ServoTeleop extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;

    @Override
    public void runOpMode() {
        // Initialize the servos
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Get the Y-axis value of the left stick
            double forwardValue = gamepad1.left_stick_y * 0.25;

            // Control the servos based on the joystick input
            if (forwardValue > 0) {
                // Moving forward: servo1 clockwise, servo2 counterclockwise
                servo1.setPosition(1.0); // Adjust for clockwise
                servo2.setPosition(0.0); // Adjust for counterclockwise
            } else if (forwardValue < 0) {
                // Moving backward: servo1 counterclockwise, servo2 clockwise
                servo1.setPosition(0.0); // Adjust for counterclockwise
                servo2.setPosition(1.0); // Adjust for clockwise
            } else {
                // Stop the servos
                servo1.setPosition(0.5); // Neutral position
                servo2.setPosition(0.5);
            }

            // Display the servo positions in the telemetry for debugging
            telemetry.addData("Servo1 Position", servo1.getPosition());
            telemetry.addData("Servo2 Position", servo2.getPosition());
            telemetry.update();

            // Add a small delay for stability
            sleep(10);
        }
    }
}
