package org.firstinspires.ftc.teamcode.subsystem.Deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

public class DepositV4BSubsystem implements Subsystem {

    private final Servo wristServo1;
    private final Servo wristServo2;

    private static final double WRIST_1_DEFAULT = 1.0;
    private static final double WRIST_2_DEFAULT = 1.0;
    private static final double WRIST_1_DROP = 0.15;
    private static final double WRIST_2_DROP = 0.1;
    private static final double WRIST_1_PICKUP = 0.57;
    private static final double WRIST_2_PICKUP = 0.45;

    public DepositV4BSubsystem(HardwareMap hardwareMap) {
        wristServo1 = hardwareMap.get(Servo.class, "dwsrv1");
        wristServo2 = hardwareMap.get(Servo.class, "dwsrv2");
        wristServo2.setDirection(Servo.Direction.REVERSE);
    }

    public void setWristDropPosition() {
        setWristPosition(WRIST_1_DROP, WRIST_2_DROP);
    }

    public void setWristPickPosition() {
        setWristPosition(WRIST_1_PICKUP, WRIST_2_PICKUP);
    }

    public void setWristDefaultPosition() {
        setWristPosition(WRIST_1_DEFAULT, WRIST_2_DEFAULT);
    }

    private void setWristPosition(double pos1, double pos2) {
        wristServo1.setPosition(pos1);
        wristServo2.setPosition(pos2);
    }

    @Override
    public void update() {
        // No-op: Implement update logic if necessary.
    }
}
