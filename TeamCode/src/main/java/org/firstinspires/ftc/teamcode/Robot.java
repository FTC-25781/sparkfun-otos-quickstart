package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Deposit.DepositClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Deposit.DepositSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Deposit.DepositV4BSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeV4BSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;


public class Robot {
    public IntakeSubsystem intake;
    public DepositSubsystem deposit;
    public IntakeV4BSubsystem intakeV4B;
    public IntakeSlideSubsystem intakeSlide;
    public IntakeClawSubsystem intakeClaw;
    public DepositV4BSubsystem depositV4B;
    public DepositSlideSubsystem depositSlide;
    public DepositClawSubsystem depositClaw;
    public MecanumDrive drive;
    public Telemetry telemetry;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        deposit = new DepositSubsystem(hardwareMap, telemetry);
        intakeV4B = new IntakeV4BSubsystem(hardwareMap, telemetry);
        intakeSlide = new IntakeSlideSubsystem(hardwareMap, telemetry);
        intakeClaw = new IntakeClawSubsystem(hardwareMap, telemetry);
        depositV4B = new DepositV4BSubsystem(hardwareMap, telemetry);
        depositSlide = new DepositSlideSubsystem(hardwareMap, telemetry);
        depositClaw = new DepositClawSubsystem(hardwareMap, telemetry);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public void Preset() {
        this.deposit.runToPreset();
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
        this.deposit.setDepositWristDropPosition();
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