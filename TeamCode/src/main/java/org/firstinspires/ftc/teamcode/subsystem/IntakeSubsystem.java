package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

public class IntakeSubsystem implements Subsystem {
    private final DcMotor slideMotor;
    private final DcMotor verticalSlideMotor;
    private final Servo wristServo1;
    private final Servo wristServo2;
    private final Servo orientationServo;
    private final Servo clawServo;
    private final Telemetry telemetry;

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
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "verticalSlideMotor");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(0.0);
        orientationServo.setPosition(0.0);
    }

    // Main control method for individual button presses
    public void controlIntake(Gamepad gamepad) {
        if (gamepad.a) {
            openClaw();
        } else if (gamepad.b) {
            closeClaw();
        }

        if (gamepad.dpad_up) {
            extendSlide(slideMotor);
        } else if (gamepad.dpad_down) {
            retractSlide(slideMotor);
        }

        if (gamepad.dpad_right) {
            extendSlide(slideMotor);
        } else if (gamepad.dpad_left) {
            retractSlide(slideMotor);
        }

        if (gamepad.x) {
            setWristPosition(0.0, 1.0);  // Pick position
        } else if (gamepad.y) {
            setWristPosition(1.0, 0.0);  // Lift position
        } else if (gamepad.left_bumper) {
            resetWrist();  // Reset wrist to neutral position
        }

        if (gamepad.right_trigger > 0.5) {
            setOrientation(1.0);
        } else if (gamepad.left_trigger > 0.5) {
            setOrientation(0.0);
        }
    }

    // Methods to control individual actions
    private void extendSlide(DcMotor motor) {
        motor.setTargetPosition(final_pos_motor);
        motor.setPower(motor_extend_speed);
    }

    private void retractSlide(DcMotor motor) {
        motor.setTargetPosition(final_in_pos_motor);
        motor.setPower(-motor_extend_speed);
    }

    private void setWristPosition(double wrist1, double wrist2) {
        wristServo1.setPosition(wrist1);
        wristServo2.setPosition(wrist2);
    }

    private void setOrientation(double position) {
        orientationServo.setPosition(position);
        telemetry.addData("Claw Orientation", position);
        telemetry.update();
    }

    private void openClaw() {
        clawServo.setPosition(1.0);
    }

    private void closeClaw() {
        clawServo.setPosition(0.0);
    }

    private void resetWrist() {
        setWristPosition(0.5, 0.5);
    }

    // Method to stop all movements
    public void stopIntake() {
        slideMotor.setPower(0);
        clawServo.setPosition(0.0);
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    @Override
    public void update() {

    }
}
