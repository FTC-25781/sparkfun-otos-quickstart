package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Ramp Control", group = "Examples")
public class ServoRampControl extends LinearOpMode {

    private Servo myServo;
    private static final double INCREMENT = 0.01;  // Change in servo position per cycle
    private static final long CYCLE_MS = 20;       // Update period in milliseconds
    private static final double MAX_POS = 1.0;     // Maximum position
    private static final double MIN_POS = 0.0;     // Minimum position
    private double position = 0.9;                 // Starting position

    @Override
    public void runOpMode() {

        myServo = hardwareMap.get(Servo.class, "clsrv");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                position += INCREMENT;
            }

            else if (gamepad1.b) {
                position -= INCREMENT;
            }

            position = Math.max(MIN_POS, Math.min(position, MAX_POS));

            myServo.setPosition(position);

            sleep(CYCLE_MS);
            telemetry.addData("Servo Position", position);
            telemetry.update();
        }
    }
}
