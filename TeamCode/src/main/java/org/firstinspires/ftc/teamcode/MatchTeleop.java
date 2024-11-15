package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "Match Teleop", group = "Teleop")
public class MatchTeleop extends LinearOpMode {

    Robot robot;
    DcMotor slideMotor;

    MecanumDrive drive;
    DcMotor left_front, right_front, left_back, right_back;
    public double orientationPosition = 0.0;

        @Override
        public void runOpMode() {
            // Initialize the Robot and motor mappings
            robot = new Robot(hardwareMap, telemetry);

        left_front = hardwareMap.get(DcMotor.class, "left_front");  // Motor Port 2
        right_front = hardwareMap.get(DcMotor.class, "right_front"); // Motor Port 3
        left_back = hardwareMap.get(DcMotor.class, "left_back");     // Motor Port 0
        right_back = hardwareMap.get(DcMotor.class, "right_back");   // Motor Port 1
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //robot.Preset();
        waitForStart();

        while (opModeIsActive()) {
            // Game pad stick inputs for movement
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Invert Y-axis
            double rx = gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x
            ));

            setDrivePower(x, y, rx);
            robot.update();
        }
    }

    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = y - x + rx;
        double powerBackRight = y + x - rx;

        // Normalize motor powers
        double max = Math.max(Math.abs(powerFrontLeft), Math.max(Math.abs(powerFrontRight),
                Math.max(Math.abs(powerBackLeft), Math.abs(powerBackRight))));

        if (max > 1) {
            powerFrontLeft /= max;
            powerFrontRight /= max;
            powerBackLeft /= max;
            powerBackRight /= max;
        }

        // Set motor powers
        left_front.setPower(powerFrontLeft);
        right_front.setPower(powerFrontRight);
        left_back.setPower(powerBackLeft);
        right_back.setPower(powerBackRight);

        updateIntakeControls();
        updateDepositControls();
        sendTelemetry();
    }

    private void updateIntakeControls() {
        // Claw controls
        if (gamepad2.a) {
            robot.intake.openClaw();
        }
        if (gamepad2.b) {
            robot.intake.closeClaw();
        }

        // Main slide controls
        robot.intake.manualExtension(gamepad2.left_stick_y);
        robot.deposit.manualExtension(gamepad2.right_stick_y);


        // Wrist controls
        if (gamepad2.x) {
            robot.intake.setWristPickPosition();  // Pick position
        }
        if (gamepad2.y) {
            robot.intake.setWristDropPosition();  // Drop position
        }
        if (gamepad2.left_bumper) {
            robot.intake.setWristDefaultPosition();  // Neutral position
        }
        // Orientation control
        // robot.intake.setOrientation(gamepad2.right_stick_y);
    }

    private void updateDepositControls() {
        // Deposit claw controls
        if (gamepad2.dpad_up) {
            robot.deposit.openDepositClaw();
        }
        if (gamepad2.dpad_down) {
            robot.deposit.closeDepositClaw();
        }

        // Deposit slide controls
        if (gamepad2.dpad_right) {
            robot.deposit.extendDepositMainSlide();
        }
        if (gamepad2.dpad_left) {
            robot.deposit.retractDepositMainSlide();
        }

        // Deposit wrist controls
        if (gamepad2.left_trigger > 0.0) {
            robot.deposit.setDepositWristPickPosition();
        }

        if (gamepad2.right_bumper) {
            robot.deposit.setDepositWristDropPosition();
        }
    }

    private void sendTelemetry() {
        telemetry.addData("Position", orientationPosition);
        telemetry.addData("Wrist-1 position", robot.intake.wristServo1.getPosition());
        telemetry.addData("Wrist-2 position", robot.intake.wristServo2.getPosition());
        telemetry.addData("Horizontal Slide power", robot.intake.slideMotor.getPower());
        telemetry.addData("Horizontal Slide position", robot.intake.slideMotor.getCurrentPosition());



        telemetry.update();
    }
}
