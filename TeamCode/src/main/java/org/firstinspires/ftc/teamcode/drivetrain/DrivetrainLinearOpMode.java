package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DrivetrainLinearOpMode", group = "Linear Opmode")
public class DrivetrainLinearOpMode extends LinearOpMode {

    // Declare motor variables
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Set the direction of motors (reverse the right motor to avoid inversion)
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Display the status on the driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get the input from the gamepad sticks (tank drive)
            double leftPower = -gamepad1.left_stick_y;  // Left stick controls the left motor
            double rightPower = -gamepad1.right_stick_y; // Right stick controls the right motor

            // Send power to the motors
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Optionally display motor power values to the driver station
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}
