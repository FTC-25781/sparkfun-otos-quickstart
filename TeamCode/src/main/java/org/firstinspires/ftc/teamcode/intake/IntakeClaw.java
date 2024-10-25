package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeClaw {

    // Define the servo for the claw
    private Servo clawServo;

    // Define positions for open and closed states
    private static final double CLAW_OPEN_POSITION = 1.0;  // FULLY OPENED CLAW
    private static final double CLAW_CLOSED_POSITION = 0.0; // FULLY CLOSED CLAW

    // Constructor
    public IntakeClaw(HardwareMap hardwareMap) {
        // Initialize the servo from the hardware map
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    // Method to open the claw
    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    // Method to close the claw
    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED_POSITION);
    }

    // Method to set the claw to a specific position (for finer control)
    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }
}
