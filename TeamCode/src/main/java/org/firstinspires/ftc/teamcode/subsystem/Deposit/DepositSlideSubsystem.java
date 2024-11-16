package org.firstinspires.ftc.teamcode.subsystem.Deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DepositSlideSubsystem {

    private final DigitalChannel depositLimitSwitch;
    private final DcMotor verticalSlideMotor;
    private final DcMotor verticalSlideMotor2;
    private static final int SLIDE_EXTEND_POS = 800;
    private static final int SLIDE_RETRACT_POS = 0;
    private static final double SLIDE_EXTEND_SPEED = 0.5;
    private static final double MIN_SPEED = 0.1;

    public DepositSlideSubsystem(HardwareMap hardwareMap) {
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "vsmot");
        verticalSlideMotor2 = hardwareMap.get(DcMotor.class, "vsmot2");
        depositLimitSwitch = hardwareMap.get(DigitalChannel.class, "dpltsw");

        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void manualExtension(double power) {
        verticalSlideMotor.setPower(clampPower(power));
    }

    public void extendDepositMainSlide() {
        int currentPosition = verticalSlideMotor.getCurrentPosition();
        int distanceToTarget = SLIDE_EXTEND_POS - currentPosition;

        double proportionalSpeed = Math.max(MIN_SPEED, (double) distanceToTarget / 1000);
        verticalSlideMotor.setPower(distanceToTarget > 0 ? proportionalSpeed * SLIDE_EXTEND_SPEED : 0);
    }

    public void retractDepositMainSlide() {
        if (!depositLimitSwitch.getState() || verticalSlideMotor.getCurrentPosition() <= SLIDE_RETRACT_POS) {
            verticalSlideMotor.setPower(0);
            verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            verticalSlideMotor.setPower(-SLIDE_EXTEND_SPEED);
        }
    }

    private double clampPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }

    public void update() {
        // No-op: Implement any periodic updates if needed.
    }
}
