package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Deposit.DepositClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Deposit.DepositSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Deposit.DepositV4BSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeV4BSubsystem;

public class Robot {

    private final IntakeV4BSubsystem intakeV4B;
    private final IntakeSlideSubsystem intakeSlide;
    private final IntakeClawSubsystem intakeClaw;
    private final DepositV4BSubsystem depositV4B;
    private final DepositSlideSubsystem depositSlide;
    private final DepositClawSubsystem depositClaw;
    private final MecanumDrive drive;
    private final Telemetry telemetry;

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

    public void preset() {
        depositClaw.runToPreset();
        intakeClaw.runToPreset();
    }

    public void startIntakePickup() {
        intakeSlide.extendMainSlide();
        intakeClaw.openClaw();
        intakeV4B.setWristDefaultPosition();
    }

    public void startIntakeDrop() {
        intakeV4B.wristPositionAction();
        intakeClaw.closeClaw();
        intakeV4B.setWristDropPosition();
        intakeSlide.retractMainSlide();
    }

    public void startDepositPickup() {
        intakeClaw.openClaw();
        intakeV4B.setWristDefaultPosition();
        depositClaw.openDepositClaw();
        depositV4B.setWristPickPosition();
        depositSlide.retractDepositMainSlide();
        depositClaw.closeDepositClaw();
    }

    public void startDepositDrop() {
        depositSlide.extendDepositMainSlide();
        depositV4B.setWristDropPosition();
        depositClaw.openDepositClaw();
        depositClaw.closeDepositClaw();
    }

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
