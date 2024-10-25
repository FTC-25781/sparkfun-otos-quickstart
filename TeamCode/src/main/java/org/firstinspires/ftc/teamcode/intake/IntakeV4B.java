package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Virtual 4-Bar Intake", group = "")
public class IntakeV4B extends LinearOpMode {

    // Declare hardware components
    private Servo whole4Barservo;
    private Servo ClawServo;

    @Override
    public void runOpMode() {
        // Initialize the hardware components
        ClawServo = hardwareMap.get(Servo.class, "intakeServo");


        // Set starting position for the servo
        ClawServo.setPosition(0);


        // Wait for start to be pressed
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // Control the WHOLE virtual 4-bar with gamepad left stick
            double power = -gamepad1.left_stick_y;  // Invert if necessary for proper direction
            ClawServo.setPosition(0);
            ClawServo.setPosition(1);

            // Control the INTAKE (claw) servo with gamepad buttons
            if (gamepad1.a) {
                whole4Barservo.setPosition(1);  // Turn on the intake
            } else if (gamepad1.b) {
                whole4Barservo.setPosition(0);  // Turn off the intake
            }

            // Telemetry to display motor and servo status
            telemetry.addData("Left Motor Power", whole4Barservo.getPosition());
            telemetry.addData("Servo Position", ClawServo.getPosition());
            telemetry.update();
        }
    }
}


