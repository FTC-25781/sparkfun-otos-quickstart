package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

public class IntakeSubsystem {
    private final DcMotor slideMotor;
    private final Servo wristServo1;
    private final Servo wristServo2;
    private final Servo orientationServo;
    private final Servo clawServo;
    private final Telemetry telemetry;

    VisionAngleSub servoOrientation;

    int final_pos_motor = 800;
    int final_in_pos_motor = 0;
    double motor_extend_speed = 0.5;

    // Constructor for initializing the subsystem
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        wristServo1 = hardwareMap.get(Servo.class, "wristServo1");
        wristServo2 = hardwareMap.get(Servo.class, "wristServo2");
        orientationServo = hardwareMap.get(Servo.class, "orientationServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Zero Power Behavior to BRAKE
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoOrientation = new VisionAngleSub();
        clawServo.setPosition(0.0);
        orientationServo.setPosition(0.0);
    }

    // Main method to start the intake, calling subfunctions
    public void startIntake(Gamepad gamepad) {
        if (gamepad.a) {
            extendSlide();
            setWristPosition(0.5, 0.5); // Initial wrist position
            setOrientation();
            openClaw();
            setWristPosition(0.0, 1.0); // Pick position
            closeClaw();
            setWristPosition(1.0, 0.0); // Lift wrist for drop
            openClaw();
            resetWrist();
            retractSlide();
        }
    }

    // 1. Slide extending outward
    private void extendSlide() {
        slideMotor.setTargetPosition(final_pos_motor);
        slideMotor.setPower(motor_extend_speed);
        while (slideMotor.isBusy()) {
            telemetry.addData("Current Position", slideMotor.getCurrentPosition());
            telemetry.update();
        }
        slideMotor.setPower(0);
    }

    // 2. Set wrist position
    private void setWristPosition(double wrist1, double wrist2) {
        wristServo1.setPosition(wrist1);
        wristServo2.setPosition(wrist2);
    }

    // 3. Get and set the orientation value
    private void setOrientation() {
        double orientation = servoOrientation.getOrientation();
        orientationServo.setPosition(orientation);
        telemetry.addData("Claw Orientation", orientation);
        telemetry.update();
    }

    // 4. Open claw
    private void openClaw() {
        clawServo.setPosition(1.0);
    }

    // 6. Close claw
    private void closeClaw() {
        clawServo.setPosition(0.0);
    }

    // 9. Reset wrist to neutral position
    private void resetWrist() {
        setWristPosition(0.5, 0.5);
    }

    // 10. Slide retracting inward
    private void retractSlide() {
        slideMotor.setTargetPosition(final_in_pos_motor);
        slideMotor.setPower(-motor_extend_speed);
    }

    // Method to stop the intake
    public void stopIntake() {
        slideMotor.setPower(0);
        clawServo.setPosition(0.0);
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }
}
