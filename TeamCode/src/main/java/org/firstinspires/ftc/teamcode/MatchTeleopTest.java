package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Teleop")
public class MatchTeleopTest extends LinearOpMode {

    Robot robot;
    DcMotor left_front, right_front, left_back, right_back, slideMotor;
    public double orientationPosition = 0.0;
    public Servo wristServo1;
    public Servo wristServo2;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);

        wristServo1 = hardwareMap.get(Servo.class, "wsrv1"); // Servo Port 2
        wristServo2 = hardwareMap.get(Servo.class, "wsrv2"); // Servo Port 3

        wristServo2.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.dpad_right) {
                wristServo1.setPosition(0.1);
                wristServo2.setPosition(0.1);
            }

            if (gamepad2.dpad_left) {
                wristServo1.setPosition(0.3);
                wristServo2.setPosition(0.3);
            }

            if (gamepad2.dpad_up) {
                wristServo1.setPosition(0.05);
                wristServo2.setPosition(0.05);
            }

            telemetry.addData("Servo1 pos", wristServo1.getPosition());
            telemetry.addData("Servo2 pos", wristServo2.getPosition());
            telemetry.update();
        }
    }
}
