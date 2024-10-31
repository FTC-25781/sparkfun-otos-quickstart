package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeTest implements Subsystem {
    private final DcMotor slideMotor;
    public final Servo wristServo1;
    public final Servo wristServo2;
    private final Servo orientationServo;
    private final Servo clawServo;
    private final Telemetry telemetry;
    private final DigitalChannel intakeLimitSwitch;

    final int SLIDE_EXTEND_POS = 800;
    final int SLIDE_RETRACT_POS = 0;
    final double SLIDE_EXTEND_SPEED = 0.5;

    // Add three wrist positions
    final double WRIST_UP_POS = 0.79;
    final double WRIST_DEFAULT_POS = 0.5;
    final double WRIST_DOWN_POS = 0.4;

    // Add two claw positions
    final double CLAW_OPEN_POS = 0.69;
    final double CLAW_CLOSED_POS = 0.9;

    final double ORIENTATION_DEFAULT_POS = 0.0;

    // Constructor for initializing the subsystem
    public IntakeTest(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        slideMotor = hardwareMap.get(DcMotor.class, "hsmot"); // Motor Port 0
        wristServo1 = hardwareMap.get(Servo.class, "wsrv1"); // Servo Port 2
        wristServo2 = hardwareMap.get(Servo.class, "wsrv2"); // Servo Port 3
        orientationServo = hardwareMap.get(Servo.class, "orsrv"); // Servo Port  0
        clawServo = hardwareMap.get(Servo.class, "clsrv"); // Servo Port 1
        intakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "inltsw"); // Digital Port 0

        // Set the limit switch to INPUT mode
        intakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runToPreset() {
        clawServo.setPosition(CLAW_CLOSED_POS);
        // TODO: Make sure orientation is reachable
        setOrientation(ORIENTATION_DEFAULT_POS);
        retractMainSlide();
    }

    // Slide Functions
    public void maunualExtention(double y) {
        slideMotor.setPower(y);
        telemetry.addData("Slide Extending", y);
        telemetry.update();
    }

    public void extendMainSlide() {
        // TODO: Find slide full extend position
        slideMotor.setTargetPosition(SLIDE_EXTEND_POS);
        slideMotor.setPower(SLIDE_EXTEND_SPEED);
    }

    public void retractMainSlide() {
        if (!intakeLimitSwitch.getState()) {
            slideMotor.setPower(0);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            slideMotor.setPower(-SLIDE_EXTEND_SPEED);
        }
    }

    // Wrist Functions
    public void setWristDropPosition() {
        wristServo1.setPosition(WRIST_UP_POS);
        wristServo2.setPosition(WRIST_DOWN_POS);

    }

    public void setWristPickPosition() {
        wristServo1.setPosition(WRIST_DOWN_POS);
        wristServo2.setPosition(WRIST_UP_POS);
    }

    public void setWristDefaultPosition() {
        wristServo1.setPosition(WRIST_DEFAULT_POS);
        wristServo2.setPosition(WRIST_DEFAULT_POS);

    }

    public double getWrist2Postion() {
        return wristServo2.getPosition();
    }

    // Orientation Functions
    public void setOrientation(double position) {
        orientationServo.setPosition(position);
        telemetry.addData("Claw Orientation", position);
        telemetry.update();
    }

    // Claw Functions
    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    // Method to stop all movements
    public void intakeEmergencyStop() {
        slideMotor.setPower(0);
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    @Override
    public void update() {

    }
}
