package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSubsystem {
    private DcMotor outtakeMotor;

    // Constants
    private static final double OUTTAKE_POWER = 1.0; // Adjust this power level as needed

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        // Initialize the outtake motor
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtake_motor"); // Replace with your motor's name
        outtakeMotor.setDirection(DcMotor.Direction.REVERSE); // Adjust if needed
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Optional: Set behavior when power is 0
    }

    public OuttakeSubsystem() {
    }

    // Method to run the outtake
    public void runOuttake() {
        outtakeMotor.setPower(OUTTAKE_POWER);
    }

    // Method to stop the outtake
    public void stopOuttake() {
        outtakeMotor.setPower(0);
    }

    // Method to control outtake with gamepad
    public void controlOuttake(Gamepad gamepad) {
        if (gamepad.right_bumper) { // Use right bumper to activate outtake
            runOuttake();
        } else {
            stopOuttake();
        }
    }
}
