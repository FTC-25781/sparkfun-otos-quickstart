package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrainSubsystem {
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor rearLeftMotor;
    private final DcMotor rearRightMotor;

    public MecanumDrivetrainSubsystem(HardwareMap hardwareMap) {
        // Initialize the motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rear_right_drive");

        // Set motor directions (adjust as necessary based on your wiring)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to brake when power is set to zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Method to calculate and set motor power for mecanum drive
    public void drive(double y, double x, double rotate) {
        // Adjust for strafing (x input) to compensate for wheel orientation
        double strafe = x * 1.1;

        // Calculate power for each motor based on mecanum formula
        double frontLeftPower = y + strafe + rotate;
        double frontRightPower = y - strafe - rotate;
        double rearLeftPower = y - strafe + rotate;
        double rearRightPower = y + strafe - rotate;

        // Set power to each motor
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        rearLeftMotor.setPower(rearLeftPower);
        rearRightMotor.setPower(rearRightPower);
    }

    // Method to stop the motors
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }
}
