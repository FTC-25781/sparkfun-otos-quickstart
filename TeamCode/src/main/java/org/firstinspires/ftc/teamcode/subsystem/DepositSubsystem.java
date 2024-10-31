package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DepositSubsystem implements Subsystem {
    private final DcMotor verticalSlideMotor;
    private final Servo wristServo1;
    private final Servo wristServo2;
    private final Servo clawServo;
    private final Telemetry telemetry;
    private final DigitalChannel intakeLimitSwitch;

    final int SLIDE_EXTEND_POS = 800;
    final int SLIDE_RETRACT_POS = 0;
    final double SLIDE_EXTEND_SPEED = 0.5;

    // Add three wrist positions
    final double WRIST_UP_POS = 1.0;
//    final double WRIST_DEFAULT_POS = 0.5;
    final double WRIST_DOWN_POS = 0.0;

    // Add two claw positions
    final double CLAW_OPEN_POS = 0.22;
    final double CLAW_CLOSED_POS = 0.52;

    // Constructor for initializing the subsystem
    public DepositSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "vsmot"); // Motor Port 1
        wristServo1 = hardwareMap.get(Servo.class, "dwsrv1"); // Servo Port 1
        wristServo2 = hardwareMap.get(Servo.class, "dwsrv2"); // Servo Port 2
        clawServo = hardwareMap.get(Servo.class, "dclsrv"); // Servo Port 3
        intakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "dpltsw"); // Digital Port 0

        // Set the limit switch to INPUT mode
        intakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    public void runToPreset() {
        clawServo.setPosition(CLAW_CLOSED_POS);
        // TODO: Make sure orientation is reachable
        retractDepositMainSlide();
        setDepositWristLiftPosition();
    }

    // Methods to control individual actions
    public void extendDepositMainSlide() {
        verticalSlideMotor.setTargetPosition(SLIDE_EXTEND_POS);
        verticalSlideMotor.setPower(SLIDE_EXTEND_SPEED);
    }

    public void retractDepositMainSlide() {
        if (!intakeLimitSwitch.getState()) {
            verticalSlideMotor.setPower(0);
            verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            verticalSlideMotor.setPower(-SLIDE_EXTEND_SPEED);
        }
    }
    public void setDepositWristPickPosition() {
        wristServo1.setPosition(WRIST_DOWN_POS);
        wristServo2.setPosition(WRIST_UP_POS);
    }

    public void setDepositWristLiftPosition() {
        wristServo1.setPosition(WRIST_UP_POS);
        wristServo2.setPosition(WRIST_DOWN_POS);
    }

    public void openDepositClaw() {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    public void closeDepositClaw() {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

// TODO: Make sure we need this code
/*    public void resetWrist() {
        wristServo1.setPosition(WRIST_DEFAULT_POS);
        wristServo2.setPosition(WRIST_DEFAULT_POS);
    } yey */

    // Method to stop all movements
    public void stopDepositIntake() {
        verticalSlideMotor.setPower(0);
        clawServo.setPosition(CLAW_CLOSED_POS);
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    @Override
    public void update() {

    }
}
