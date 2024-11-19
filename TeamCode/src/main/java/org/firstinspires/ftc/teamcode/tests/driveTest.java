package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Drive", group="TeleOp")
public class driveTest extends OpMode {

    // Declare motor variables
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void init() {
        // Initialize motors from the hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions to ensure correct movement
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Get gamepad input
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x * 1.1;  // Strafing with adjustment
        double rotation = gamepad1.right_stick_x;  // Rotation

        // Calculate power for each motor
        double frontLeftPower = y + x + rotation;
        double frontRightPower = y - x - rotation;
        double backLeftPower = y - x + rotation;
        double backRightPower = y + x - rotation;

        // Normalize power values to keep them within [-1, 1]
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // Telemetry feedback for debugging
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.update();
    }
}
