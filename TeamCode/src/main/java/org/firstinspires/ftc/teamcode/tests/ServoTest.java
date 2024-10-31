package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo", group = "Teleop")
public class ServoTest extends LinearOpMode {
    private Servo servo;

    @Override
    public void runOpMode() {
        // Initialize the servo from the hardware map
        servo = hardwareMap.get(Servo.class, "test");

        // Wait for the start button to be pressed
        waitForStart();

        // Main loop during teleop mode
        while (opModeIsActive()) {
            // Set servo position
            servo.setPosition(0.0);

            // Add telemetry for debugging, if needed
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}
