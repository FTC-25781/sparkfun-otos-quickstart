package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeClawSubsystem {
    private final Telemetry telemetry;
    private Servo clawServo;

    public IntakeClawSubsystem() {
        clawServo = null;
        telemetry = null;
    }

    public IntakeClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Servo orientationServo) {
        this.telemetry = telemetry;
        clawServo = hardwareMap.get(Servo.class, "clsrv"); // Servo Port 1
        orientationServo = hardwareMap.get(Servo.class, "orsrv"); // Servo Port  0
    }

    public void openClaw(double CLAW_OPEN_POS) {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    public void closeClaw(double CLAW_CLOSED_POS) {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    public void update() {

    }

}

