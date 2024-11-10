package org.firstinspires.ftc.teamcode.subsystem.Deposit;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

public class DepositClawSubsystem implements Subsystem {
    private final Servo clawServo;
    private final Telemetry telemetry;

    final double CLAW_OPEN_POS = 0.42;
    final double CLAW_CLOSED_POS = 0.52;

    private double WRIST_1_DEFAULT = 1.0;
    private double WRIST_2_DEFAULT = 1.0;

    private double WRIST_1_DROP = 0.15;
    private double WRIST_2_DROP = 0.1;
    private double WRIST_1_PICKUP = 0.57;
    private double WRIST_2_PICKUP = 0.45;

    public DepositClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.get(Servo.class, "dclsrv"); // Servo Port 3

        clawServo.setPosition(CLAW_CLOSED_POS);
    }

}







