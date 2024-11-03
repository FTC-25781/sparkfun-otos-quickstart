package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DepositSubsystem implements Subsystem {
    private final DcMotor verticalSlideMotor;
    public final Servo wristServo1;
    public final Servo wristServo2;
    private final Servo clawServo;
    private final Telemetry telemetry;
    private final DigitalChannel intakeLimitSwitch;

    final int SLIDE_EXTEND_POS = 800;
    final int SLIDE_RETRACT_POS = 0;
    final double SLIDE_EXTEND_SPEED = 0.5;

    // Claw positions
    final double CLAW_OPEN_POS = 0.42;
    final double CLAW_CLOSED_POS = 0.52;

    private double WRIST_1_DEFAULT = 1.0;
    private double WRIST_2_DEFAULT = 1.0;

    private double WRIST_1_DROP = 0.15;
    private double WRIST_2_DROP = 0.1;
    private double WRIST_1_PICKUP = 0.57;
    private double WRIST_2_PICKUP = 0.45;

    // Constructor for initializing the subsystem
    public DepositSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "vsmot"); // Motor Port 1
        wristServo1 = hardwareMap.get(Servo.class, "dwsrv1"); // Servo Port 1
        wristServo2 = hardwareMap.get(Servo.class, "dwsrv2"); // Servo Port 0
        clawServo = hardwareMap.get(Servo.class, "dclsrv"); // Servo Port 3
        intakeLimitSwitch = hardwareMap.get(DigitalChannel.class, "dpltsw"); // Digital Port 0

        wristServo2.setDirection(Servo.Direction.REVERSE);

        // Set the limit switch to INPUT mode
        intakeLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        //verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    public void runToPreset() {
        clawServo.setPosition(CLAW_CLOSED_POS);
//        retractDepositMainSlide();
//
    }

    public void manualExtension(double y) {
        verticalSlideMotor.setPower(y); // Clamp to valid motor power range
    }

    // Slide control methods
    public void extendDepositMainSlide() {
        int currentPosition = verticalSlideMotor.getCurrentPosition();
        int distanceToTarget = SLIDE_EXTEND_POS - currentPosition;

        // Basic proportional control (adjust the divisor for sensitivity)
        double proportionalSpeed = Math.max(0.1, distanceToTarget / 1000.0); // Min speed of 0.1

        if (distanceToTarget > 0) {
            verticalSlideMotor.setPower(proportionalSpeed * SLIDE_EXTEND_SPEED);
        } else {
            verticalSlideMotor.setPower(0);
        }
    }

    public void retractDepositMainSlide() {
        if (!intakeLimitSwitch.getState() && verticalSlideMotor.getCurrentPosition() != SLIDE_RETRACT_POS) {
            verticalSlideMotor.setPower(0);
            verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            verticalSlideMotor.setPower(-SLIDE_EXTEND_SPEED);
        }
    }

    // Wrist control methods
    public void setDepositWristDropPosition() {
        wristServo1.setPosition(WRIST_1_DROP);
        wristServo2.setPosition(WRIST_2_DROP);
    }

    public void setDepositWristPickPosition() {
        wristServo1.setPosition(WRIST_1_PICKUP);
        wristServo2.setPosition(WRIST_2_PICKUP);
    }

    public void setDepositWristDefaultPosition() {
        wristServo1.setPosition(WRIST_1_DEFAULT);
        wristServo2.setPosition(WRIST_2_DEFAULT);
    }

    // Claw control methods
    public void openDepositClaw() {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    public void closeDepositClaw() {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    // Method to stop all movements
    public void stopDepositIntake() {
        verticalSlideMotor.setPower(0);
        clawServo.setPosition(CLAW_CLOSED_POS);
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    @Override
    public void update() {
        // Placeholder for periodic updates if needed
    }
}
