package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class Robot {
    private final IntakeSubsystem intake;
    private final DepositSubsystem deposit;
    private final MecanumDrive drive;
    private final Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        deposit = new DepositSubsystem(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public void controlIntake(Gamepad gamepad) {

        // Main slide controls
        if (gamepad.dpad_up) {
            intake.extendMainSlide();
        } else if (gamepad.dpad_down) {
            intake.retractMainSlide();
        }

        // Claw controls
        if (gamepad.a) {
            intake.openClaw();
        } else if (gamepad.b) {
            intake.closeClaw();
        }

        //Claw Orientation controls
        double orientationPosition = gamepad.right_trigger > 0 ? gamepad.right_trigger : gamepad.left_trigger;
        intake.setOrientation(orientationPosition);

        // Wrist controls
        if (gamepad.x) {
            intake.setWristPickPosition();  // Pick position
        } else if (gamepad.y) {
            intake.setWristLiftPosition();  // Lift position
        } else if (gamepad.left_bumper) {
            intake.resetWrist();  // Reset wrist to neutral position
        }

        //Deposit claw controls
        if (gamepad.left_stick_y > 0) {
            deposit.openDepositClaw();
        } else if (gamepad.left_stick_y < 0) {
            deposit.closeDepositClaw();
        }

        // Deposit slide controls
        if (gamepad.dpad_right) {
            deposit.extendDepositMainSlide();
        } else if (gamepad.dpad_left) {
            deposit.retractDepositMainSlide();
        }

        if (gamepad.right_stick_y > 0.0) {
            deposit.setDepositWristPickPosition();  // Pick position
        } else if (gamepad.right_bumper) {
            deposit.setDepositWristLiftPosition();  // Lift position
        }

    }

    // Method to update all subsystems
    public void update() {
        intake.update();
        deposit.update();
        drive.updatePoseEstimate();
    }
}