package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSlideSubsystem {

    private final DcMotor slideMotor;
    private final DigitalChannel intakeLimitSwitch;

    private static final int SLIDE_EXTEND_POS = 800;
    private static final double SLIDE_EXTEND_SPEED = 0.5;

    public IntakeSlideSubsystem(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, "hsmot");
        intakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "inltsw");

        intakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void manualExtension(double power) {
        slideMotor.setPower(clampMotorPower(power));
    }

    public void extendMainSlide() {
        slideMotor.setTargetPosition(SLIDE_EXTEND_POS);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDE_EXTEND_SPEED);
    }

    public void retractMainSlide() {
        if (!intakeLimitSwitch.getState()) {
            stopAndResetSlide();
        } else {
            slideMotor.setPower(-SLIDE_EXTEND_SPEED);
        }
    }

    private void stopAndResetSlide() {
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private double clampMotorPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }

    public void update() {
        // Placeholder for periodic updates if needed
    }
}
