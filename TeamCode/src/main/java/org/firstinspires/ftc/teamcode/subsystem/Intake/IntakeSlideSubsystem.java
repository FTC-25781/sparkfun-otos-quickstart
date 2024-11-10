package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSlideSubsystem {
    public DcMotor slideMotor;
    final int SLIDE_EXTEND_POS = 800;
    final double SLIDE_EXTEND_SPEED = 0.5;

    public IntakeSlideSubsystem(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "hsmot"); // Motor Port 0
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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