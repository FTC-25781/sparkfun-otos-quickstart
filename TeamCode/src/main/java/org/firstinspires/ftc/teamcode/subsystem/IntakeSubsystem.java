package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem implements Subsystem {
    public final DcMotor slideMotor;
    public final Servo wristServo1;
    public final Servo wristServo2;
    private final Servo orientationServo;
    private final Servo clawServo;
    private final Telemetry telemetry;
    private final DigitalChannel intakeLimitSwitch;

    final int SLIDE_EXTEND_POS = 800;
    final double SLIDE_EXTEND_SPEED = 0.5;

    // Claw positions
    final double CLAW_OPEN_POS = 0.69;
    final double CLAW_CLOSED_POS = 0.9;

    final double ORIENTATION_DEFAULT_POS = 0.0;

    // Constructor for initializing the subsystem
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        slideMotor = hardwareMap.get(DcMotor.class, "hsmot"); // Motor Port 0
        wristServo1 = hardwareMap.get(Servo.class, "wsrv1"); // Servo Port 2
        wristServo2 = hardwareMap.get(Servo.class, "wsrv2"); // Servo Port 3
        orientationServo = hardwareMap.get(Servo.class, "orsrv"); // Servo Port  0
        clawServo = hardwareMap.get(Servo.class, "clsrv"); // Servo Port 1
        intakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "inltsw"); // Digital Port 0

        intakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo1.setDirection(Servo.Direction.REVERSE);
    }

    public void runToPreset() {
        clawServo.setPosition(CLAW_CLOSED_POS);
        setOrientation(ORIENTATION_DEFAULT_POS);
        retractMainSlide();
    }

    // Slide Functions
    public void manualExtension(double y) {
        slideMotor.setPower(Math.max(-1, Math.min(1, y))); // Clamp to valid motor power range
    }

    public void extendMainSlide() {
        slideMotor.setTargetPosition(SLIDE_EXTEND_POS);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDE_EXTEND_SPEED);
    }

    public void retractMainSlide() {
        if (!intakeLimitSwitch.getState() && slideMotor.getCurrentPosition() != 0) {
            slideMotor.setPower(0);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            slideMotor.setPower(-SLIDE_EXTEND_SPEED);
        }
    }

    // Wrist Functions
    public void setWristDropPosition() {
        wristServo1.setPosition(0.2);
        wristServo2.setPosition(0.2);
    }

    public void setWristPickPosition() {
        wristServo1.setPosition(0.05);
        wristServo2.setPosition(0.05);
    }

    public void setWristDefaultPosition() {
        wristServo1.setPosition(0.1);
        wristServo2.setPosition(0.1);
    }

    // Orientation Functions
    public void setOrientation(double position) {
        position = Math.max(0.0, Math.min(1.0, position));  // Clamp to [0, 1]
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

    @Override
    public void update() {
        // Placeholder for periodic updates if needed
    }
}
