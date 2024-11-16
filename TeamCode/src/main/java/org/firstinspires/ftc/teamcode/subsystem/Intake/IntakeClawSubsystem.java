package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeClawSubsystem {

    private final Servo clawServo;
    private final Servo orientationServo;
    private static final double CLAW_OPEN_POS = 0.69;
    private static final double CLAW_CLOSED_POS = 0.9;

    public IntakeClawSubsystem(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clsrv");
        orientationServo = hardwareMap.get(Servo.class, "orsrv");
    }

    public void runToPreset() {
        setOrientation(1.0);
    }

    public void setOrientation(double position) {
        orientationServo.setPosition(clamp(position));
    }

    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    public void update() {
        // Placeholder for periodic updates if needed
    }
}
