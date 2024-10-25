package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends OuttakeSubsystem implements Subsystem {
    private Servo intakeMotor;
    private final double INTAKE_POWER = 1.0; // Power level for the intake
    private final double OUTTAKE_POWER = -1.0; // Power level for the outtake
    private final double STOP_POWER = 0.0; // Stop power

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = hardwareMap.get(Servo.class, "intakeMotor"); // Change "intakeMotor" to your motor's name
        intakeMotor.setPosition(STOP_POWER); // Ensure servo is stopped initially
    }

    @Override
    public void update() {

    }

    public void runIntake(Gamepad gamepad) {
        // Check if the intake button is pressed
        if (gamepad.left_trigger > 0.5) {
            intakeMotor.setPosition(INTAKE_POWER); // Run intake
        } else if (gamepad.right_trigger > 0.5) {
            intakeMotor.setPosition(OUTTAKE_POWER); // Run outtake
        } else {
            intakeMotor.setPosition(STOP_POWER); // Stop motor
        }
    }

    public void stop() {
        intakeMotor.setPosition(STOP_POWER); // Stop the intake when done
    }

    public void startIntake() {
    }

    public void reverseIntake() {
    }
}
