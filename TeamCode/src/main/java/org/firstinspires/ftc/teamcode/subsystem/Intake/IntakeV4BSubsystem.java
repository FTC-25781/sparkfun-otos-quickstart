package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeV4BSubsystem {

    public final Servo wristServo1;
    public final Servo wristServo2;

    private static final double POSITION_INCREMENT = 0.01;

    private static final double WRIST_1_DEFAULT = 0.4;
    private static final double WRIST_2_DEFAULT = 0.4;
    private static final double WRIST_1_DROP = 0.25;
    private static final double WRIST_2_DROP = 0.25;
    private static final double WRIST_1_PICKUP = 0.55;
    private static final double WRIST_2_PICKUP = 0.55;

    public IntakeV4BSubsystem(HardwareMap hardwareMap) {
        wristServo1 = hardwareMap.get(Servo.class, "wsrv1");
        wristServo2 = hardwareMap.get(Servo.class, "wsrv2");
    }

    public void setWristDropPosition() {
        setWristPosition(WRIST_1_DROP, WRIST_2_DROP);
    }

    public void setWristDefaultPosition() {
        setWristPosition(WRIST_1_DEFAULT, WRIST_2_DEFAULT);
    }

    private void setWristPosition(double pos1, double pos2) {
        wristServo1.setPosition(pos1);
        wristServo2.setPosition(pos2);
    }

    public Action wristPositionAction() {
        return new Action() {
            private double currentPos1 = wristServo1.getPosition();
            private double currentPos2 = wristServo2.getPosition();

            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                currentPos1 = adjustPosition(currentPos1, WRIST_1_PICKUP);
                currentPos2 = adjustPosition(currentPos2, WRIST_2_PICKUP);

                wristServo1.setPosition(currentPos1);
                wristServo2.setPosition(currentPos2);

                return !(isAtTarget(currentPos1, WRIST_1_PICKUP) && isAtTarget(currentPos2, WRIST_2_PICKUP));
            }

            private double adjustPosition(double current, double target) {
                return current < target
                        ? Math.min(current + POSITION_INCREMENT, target)
                        : Math.max(current - POSITION_INCREMENT, target);
            }

            private boolean isAtTarget(double current, double target) {
                return Math.abs(current - target) <= POSITION_INCREMENT;
            }
        };
    }

    public void update() {
        // Placeholder for periodic updates if needed
    }

    public void setWristPickPosition() {
    }
}
