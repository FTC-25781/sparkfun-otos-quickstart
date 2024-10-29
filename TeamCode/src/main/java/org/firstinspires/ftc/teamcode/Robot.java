package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;


public class Robot {
    public final IntakeSubsystem intake;
    public final DepositSubsystem deposit;
    public final MecanumDrive drive;
    public final Telemetry telemetry;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        deposit = new DepositSubsystem(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public void Preset() {
        this.intake.runToPreset();
    }

    public void startIntakePickup() {
        this.intake.extendMainSlide();
        this.intake.openClaw();
        this.intake.setWristDefaultPosition();
    }

    public void startIntakeDrop() {
        this.intake.setWristPickPosition();
        this.intake.closeClaw();
        this.intake.setWristDropPosition();
        this.intake.retractMainSlide();
    }


    public void startDepositPickup() {
        this.intake.openClaw();
        this.intake.setWristDefaultPosition();
        this.deposit.openDepositClaw();
        this.deposit.setDepositWristPickPosition();
        this.deposit.retractDepositMainSlide();
        this.deposit.closeDepositClaw();
    }

    public void startDepositDrop() {
        this.deposit.extendDepositMainSlide();
        this.deposit.setDepositWristLiftPosition(); //lift = drop
        this.deposit.openDepositClaw();
        this.deposit.closeDepositClaw();
    }

    // Method to update all subsystems
    public void update() {
        intake.update();
        deposit.update();
        drive.updatePoseEstimate();
    }
}