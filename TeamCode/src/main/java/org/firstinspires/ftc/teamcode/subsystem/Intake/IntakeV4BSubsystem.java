package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeV4BSubsystem {
    public Servo wristServo1;
    public Servo wristServo2;

    private static final double POSITION_INCREMENT = 0.01;

public IntakeV4BSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
    wristServo1 = hardwareMap.get(Servo.class, "wsrv1"); // Servo Port 2
    wristServo2 = hardwareMap.get(Servo.class, "wsrv2"); // Servo Port 3
}


    public void setWristPickPosition() {
        double currentPos1 = wristServo1.getPosition();
        double currentPos2 = wristServo2.getPosition();

        // Loop until both servos reach their target positions
        while (Math.abs(currentPos1 - WRIST_1_PICKUP) > POSITION_INCREMENT ||
                Math.abs(currentPos2 - WRIST_2_PICKUP) > POSITION_INCREMENT) {

            // Incrementally adjust wristServo1 position
            if (currentPos1 < WRIST_1_PICKUP) {
                currentPos1 = Math.min(currentPos1 + POSITION_INCREMENT, WRIST_1_PICKUP);
            } else if (currentPos1 > WRIST_1_PICKUP) {
                currentPos1 = Math.max(currentPos1 - POSITION_INCREMENT, WRIST_1_PICKUP);
            }
            wristServo1.setPosition(currentPos1);

            // Incrementally adjust wristServo2 position
            if (currentPos2 < WRIST_2_PICKUP) {
                currentPos2 = Math.min(currentPos2 + POSITION_INCREMENT, WRIST_2_PICKUP);
            } else if (currentPos2 > WRIST_2_PICKUP) {
                currentPos2 = Math.max(currentPos2 - POSITION_INCREMENT, WRIST_2_PICKUP);
            }
            wristServo2.setPosition(currentPos2);

            // Optional: Add a short delay for smoother movement (e.g., 20ms)
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public Action wristPositionAction() {
        return new Action() {
            double currentPos1 = wristServo1.getPosition();
            double currentPos2 = wristServo2.getPosition();

            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                if (currentPos1 < WRIST_1_PICKUP) {
                currentPos1 = Math.min(currentPos1 + POSITION_INCREMENT, WRIST_1_PICKUP);
                } else if (currentPos1 > WRIST_1_PICKUP) {
                    currentPos1 = Math.max(currentPos1 - POSITION_INCREMENT, WRIST_1_PICKUP);
                }
                wristServo1.setPosition(currentPos1);

                // Incrementally adjust wristServo2 position
                if (currentPos2 < WRIST_2_PICKUP) {
                    currentPos2 = Math.min(currentPos2 + POSITION_INCREMENT, WRIST_2_PICKUP);
                } else if (currentPos2 > WRIST_2_PICKUP) {
                    currentPos2 = Math.max(currentPos2 - POSITION_INCREMENT, WRIST_2_PICKUP);
                }
                wristServo2.setPosition(currentPos2);
                return (Math.abs(currentPos1 - WRIST_1_PICKUP) > POSITION_INCREMENT ||
                        Math.abs(currentPos2 - WRIST_2_PICKUP) > POSITION_INCREMENT);
            }
        };
    }

    public void setWristDefaultPosition() {
        wristServo1.setPosition(WRIST_1_DEFAULT);
        wristServo2.setPosition(WRIST_2_DEFAULT);
    }
    
    
    //ADDED FOR PUSHING

