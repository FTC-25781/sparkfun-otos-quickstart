package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class  IntakeClawSubsystem {

    private final Servo clawServo;
    private final Servo orientationServo;

    public IntakeClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.get(Servo.class, "clsrv");
        orientationServo = hardwareMap.get(Servo.class, "orsrv");
    }

    public void openClaw(double CLAW_OPEN_POS) {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    public void closeClaw(double CLAW_CLOSED_POS) {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    public void setOrientation(double position) {
        position = Math.max(0.0, Math.min(1.0, position));  // Clamp to [0, 1]

        // Assuming 'orientationServo' is declared and initialized elsewhere
        if (orientationServo != null) {
            orientationServo.setPosition(position);
        }
    }

    public int update() {
        return 0;
    }


}


