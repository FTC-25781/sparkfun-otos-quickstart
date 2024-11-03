package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Wrist Position Test", group="Tests")
public class WristPositionTest extends OpMode {

    private Servo wristServo1;
    private Servo wristServo2;

    private double wristPosition = 0.7;  // Default starting position

    // Constants for position limits and increment step
    private static final double WRIST_MIN_POSITION = 0.0;
    private static final double WRIST_MAX_POSITION = 1.0;
    private static final double WRIST_INCREMENT = 0.01; // Smaller increment for finer control
    private static final long WRIST_CONTROL_DELAY_MS = 100; // Delay in milliseconds

    private long lastUpdateTime = 0; // To track the last update time

    @Override
    public void init() {
        // Initialize the servos from the hardware map
        wristServo1 = hardwareMap.get(Servo.class, "wsrv1");
        wristServo2 = hardwareMap.get(Servo.class, "wsrv2");
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        // Check if enough time has passed since the last adjustment
        if (currentTime - lastUpdateTime > WRIST_CONTROL_DELAY_MS) {
            if (gamepad2.a && wristPosition < WRIST_MAX_POSITION) {
                wristPosition += WRIST_INCREMENT;  // Increase position
            } else if (gamepad2.b && wristPosition > WRIST_MIN_POSITION) {
                wristPosition -= WRIST_INCREMENT;  // Decrease position
            }

            // Update the servos to the new position
            wristServo1.setPosition(wristPosition);
            wristServo2.setPosition(wristPosition);

            // Update the last update time
            lastUpdateTime = currentTime;
        }

        // Send the current wrist position to telemetry
        telemetry.addData("Wrist Position", wristPosition);
        telemetry.update();
    }
}
