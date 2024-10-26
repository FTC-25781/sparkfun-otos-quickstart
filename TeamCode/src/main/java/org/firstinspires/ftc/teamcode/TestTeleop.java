package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Teleop", group = "Teleop")
public class TestTeleop extends LinearOpMode {

    Robot robot;
    DcMotor left_front, right_front, left_back, right_back;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);

        // Initialize motors
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        waitForStart();

        while (opModeIsActive()) {
            // Replace this with your actual input logic
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Invert Y-axis
            double rx = gamepad1.right_stick_x;

            setDrivePower(x, y, rx);
            robot.update();
        }
    }

    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        // Normalize the powers
        double max = Math.max(Math.abs(powerFrontLeft), Math.max(Math.abs(powerFrontRight),
                Math.max(Math.abs(powerBackLeft), Math.abs(powerBackRight))));

        if (max > 1) {
            powerFrontLeft /= max;
            powerFrontRight /= max;
            powerBackLeft /= max;
            powerBackRight /= max;
        }

        left_front.setPower(powerFrontLeft);
        right_front.setPower(powerFrontRight);
        left_back.setPower(powerBackLeft);
        right_back.setPower(powerBackRight);
    }
}
