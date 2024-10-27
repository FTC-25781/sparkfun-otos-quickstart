package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem implements Subsystem {
    private final DcMotor slideMotor;
    private final Servo wristServo1;
    private final Servo wristServo2;
    private final Servo orientationServo;
    private final Servo clawServo;
    private final Telemetry telemetry;
    private final DigitalChannel intakeLimitSwitch;

    final int SLIDE_EXTEND_POS = 800;
    final int SLIDE_RETRACT_POS = 0;
    final double SLIDE_EXTEND_SPEED = 0.5;

    // Add three wrist positions
    final double WRIST_UP_POS = 1.0;
    final double WRIST_DEFAULT_POS = 0.5;
    final double WRIST_DOWN_POS = 0.0;

    // Add two claw positions
    final double CLAW_OPEN_POS = 0.69;
    final double CLAW_CLOSED_POS = 0.9;

    // Constructor for initializing the subsystem
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        slideMotor = hardwareMap.get(DcMotor.class, "hsmot");
        wristServo1 = hardwareMap.get(Servo.class, "wsrv1");
        wristServo2 = hardwareMap.get(Servo.class, "wsrv2");
        orientationServo = hardwareMap.get(Servo.class, "orsrv");
        clawServo = hardwareMap.get(Servo.class, "clsrv");
        intakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "ikltsw");

        // Set the limit switch to INPUT mode
        intakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(CLAW_CLOSED_POS);
        orientationServo.setPosition(0.0);
    }

    public void extendMainSlide() {
        if (!intakeLimitSwitch.getState()) {
            slideMotor.setPower(0);
        } else {
            slideMotor.setTargetPosition(SLIDE_EXTEND_POS);
            slideMotor.setPower(SLIDE_EXTEND_SPEED);
        }
    }

    public void retractMainSlide() {
        slideMotor.setTargetPosition(SLIDE_RETRACT_POS);
        slideMotor.setPower(-SLIDE_EXTEND_SPEED);
    }
    public void setWristPickPosition() {
        wristServo1.setPosition(WRIST_DOWN_POS);
        wristServo2.setPosition(WRIST_UP_POS);
    }

    public void setWristLiftPosition() {
        wristServo1.setPosition(WRIST_UP_POS);
        wristServo2.setPosition(WRIST_DOWN_POS);
    }

    public void setOrientation(double position) {
        orientationServo.setPosition(position);
        telemetry.addData("Claw Orientation", position);
        telemetry.update();
    }

    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    public void resetWrist() {
        wristServo1.setPosition(WRIST_DEFAULT_POS);
        wristServo2.setPosition(WRIST_DEFAULT_POS);
    }

    // Method to stop all movements
    public void stopIntake() {
        slideMotor.setPower(0);
        clawServo.setPosition(CLAW_CLOSED_POS);
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    @Override
    public void update() {

    }
}
