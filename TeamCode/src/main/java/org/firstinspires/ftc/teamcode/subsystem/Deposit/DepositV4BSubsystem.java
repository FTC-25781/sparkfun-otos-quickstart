package org.firstinspires.ftc.teamcode.subsystem.Deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

public class DepositV4BSubsystem implements Subsystem {
    public Servo wristServo1;
    public Servo wristServo2;
    private final double WRIST_1_DEFAULT = 1.0;
    private final double WRIST_2_DEFAULT = 1.0;

    private final double WRIST_1_DROP = 0.15;
    private final double WRIST_2_DROP = 0.1;
    private final double WRIST_1_PICKUP = 0.57;
    private final double WRIST_2_PICKUP = 0.45;

    public void DepositSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        wristServo1 = hardwareMap.get(Servo.class, "dwsrv1"); // Servo Port 1
        wristServo2 = hardwareMap.get(Servo.class, "dwsrv2"); // Servo Port 0
    }

    public void setDepositWristDropPosition() {
        wristServo1.setPosition(WRIST_1_DROP);
        wristServo2.setPosition(WRIST_2_DROP);
    }

    public void setDepositWristPickPosition() {
        wristServo1.setPosition(WRIST_1_PICKUP);
        wristServo2.setPosition(WRIST_2_PICKUP);
    }

    public void setDepositWristDefaultPosition() {
        wristServo1.setPosition(WRIST_1_DEFAULT);
        wristServo2.setPosition(WRIST_2_DEFAULT);
    }


    @Override
    public void update() {

    }
}
