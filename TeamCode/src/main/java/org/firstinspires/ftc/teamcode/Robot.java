package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

public class Robot {

    IntakeSubsystem intake;

    OuttakeSubsystem outtake;

    MecanumDrive drive;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }

    public void update() {
        intake.update();
        outtake.update();
        drive.updatePoseEstimate();
    }

}
