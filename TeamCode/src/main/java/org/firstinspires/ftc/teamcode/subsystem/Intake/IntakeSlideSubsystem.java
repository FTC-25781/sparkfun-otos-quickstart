package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSlideSubsystem {
    public DcMotor slideMotor;
    public DcMotor slideMotor2;
    private DigitalChannel intakeLimitSwitch;

    final int SLIDE_EXTEND_POS = 800;
    final double SLIDE_EXTEND_SPEED = 0.5;

    public IntakeSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        slideMotor = hardwareMap.get(DcMotor.class, "hsmot"); // Motor Port 0
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2 = hardwareMap.get(DcMotor.class, "hsmot2");
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "inltsw"); // Digital Port 0

        intakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

    }

    public void manualExtension(double y) {
        slideMotor.setPower(y); // Clamp to valid motor power range

    }

    public void extendMainSlide() {
        slideMotor.setTargetPosition(SLIDE_EXTEND_POS);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDE_EXTEND_SPEED);


    }

    public void retractMainSlide(){}
}
