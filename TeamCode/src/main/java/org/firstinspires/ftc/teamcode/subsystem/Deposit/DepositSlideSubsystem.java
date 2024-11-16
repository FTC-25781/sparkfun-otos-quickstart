package org.firstinspires.ftc.teamcode.subsystem.Deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DepositSlideSubsystem {
    private DigitalChannel intakeLimitSwitch;
    private DcMotor verticalSlideMotor;
    private DcMotor verticalSlideMotor2;

    final int SLIDE_EXTEND_POS = 800;
    final int SLIDE_RETRACT_POS = 0;
    final double SLIDE_EXTEND_SPEED = 0.5;

    public DepositSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "vsmot"); // Motor Port 1
        intakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "dpltsw"); // Digital Port 0
        verticalSlideMotor2 = hardwareMap.get(DcMotor.class, "vsmot2");
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void retractDepositMainSlide() {
        if (verticalSlideMotor.getCurrentPosition() != SLIDE_RETRACT_POS) {
            verticalSlideMotor.setTargetPosition(SLIDE_RETRACT_POS);
            verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            verticalSlideMotor.setPower(-SLIDE_EXTEND_SPEED);

        } else {

            verticalSlideMotor.setPower(0);
        }
    }

    public void update() {

    }

}