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
        intakeV4B = new IntakeV4BSubsystem(hardwareMap);
        intakeSlide = new IntakeSlideSubsystem(hardwareMap);
        intakeClaw = new IntakeClawSubsystem(hardwareMap);
        depositV4B = new DepositV4BSubsystem(hardwareMap);
        depositSlide = new DepositSlideSubsystem(hardwareMap);
        depositClaw = new DepositClawSubsystem(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public void Preset() {
        this.depositClaw.runToPreset();
        this.intakeClaw.runToPreset();
    }

    public void startIntakePickup() {
        this.intakeSlide.extendMainSlide();
        this.intakeClaw.openClaw();
        this.intakeV4B.setWristDefaultPosition();
    }

    public void startIntakeDrop() {
        this.intakeV4B.wristPositionAction();
        this.intakeClaw.closeClaw();
        this.intakeV4B.setWristDropPosition();
        this.intakeSlide.retractMainSlide();
    }


    public void startDepositPickup() {
        this.intakeClaw.openClaw();
        this.intakeV4B.setWristDefaultPosition();
        this.depositClaw.openDepositClaw();
        this.depositV4B.setWristPickPosition();
        this.depositSlide.retractDepositMainSlide();
        this.depositClaw.closeDepositClaw();
    }

    public void startDepositDrop() {
        this.depositSlide.extendDepositMainSlide();
        this.depositV4B.setWristDropPosition();
        this.depositClaw.openDepositClaw();
        this.depositClaw.closeDepositClaw();
    }

    // Method to update all subsystems
    public void update() {
        intakeClaw.update();
        intakeSlide.update();
        intakeV4B.update();
        depositClaw.update();
        depositSlide.update();
        depositV4B.update();
        drive.updatePoseEstimate();
    }
}