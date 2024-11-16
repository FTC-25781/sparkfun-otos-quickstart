package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Ramp Control", group = "Examples")
public class ServoRampControl extends LinearOpMode {

     Servo myServo;

    @Override
    public void runOpMode() {
        myServo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {
            myServo.setPosition(0.5);

            // Update telemetry within the loop
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.update();
        }
    }
}
