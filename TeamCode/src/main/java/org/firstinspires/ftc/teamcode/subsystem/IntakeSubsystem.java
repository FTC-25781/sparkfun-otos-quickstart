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

    private static final double POSITION_INCREMENT = 0.01;
//
    private double WRIST_1_DEFAULT = 0.4;
    private double WRIST_2_DEFAULT = 0.4;

    private double WRIST_1_DROP = 0.25;
    private double WRIST_2_DROP = 0.25;
    private double WRIST_1_PICKUP = 0.55;
    private double WRIST_2_PICKUP = 0.55;

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

        //slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo1.setDirection(Servo.Direction.REVERSE);
    }

    public void runToPreset() {
//        clawServo.setPosition(CLAW_CLOSED_POS);
        setOrientation(1.0);
        //retractMainSlide();
    }

    // Slide Functions
    public void manualExtension(double y) {
        slideMotor.setPower(y); // Clamp to valid motor power range
    }

    public void extendMainSlide() {
        slideMotor.setTargetPosition(SLIDE_EXTEND_POS);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(SLIDE_EXTEND_SPEED);
    }

    public void retractMainSlide() {
//        if (!intakeLimitSwitch.getState() && slideMotor.getCurrentPosition() != 0) {
//            slideMotor.setPower(0);
//            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        } else {
//            slideMotor.setPower(-SLIDE_EXTEND_SPEED);
//        }
    }

    // Wrist Functions
    public void setWristDropPosition() {
        wristServo1.setPosition(WRIST_1_DROP);
        wristServo2.setPosition(WRIST_2_DROP);
    }


    public void setWristPickPosition() {
        double currentPos1 = wristServo1.getPosition();
        double currentPos2 = wristServo2.getPosition();

        // Loop until both servos reach their target positions
        while (Math.abs(currentPos1 - WRIST_1_PICKUP) > POSITION_INCREMENT ||
                Math.abs(currentPos2 - WRIST_2_PICKUP) > POSITION_INCREMENT) {

            // Incrementally adjust wristServo1 position
            if (currentPos1 < WRIST_1_PICKUP) {
                currentPos1 = Math.min(currentPos1 + POSITION_INCREMENT, WRIST_1_PICKUP);
            } else if (currentPos1 > WRIST_1_PICKUP) {
                currentPos1 = Math.max(currentPos1 - POSITION_INCREMENT, WRIST_1_PICKUP);
            }
            wristServo1.setPosition(currentPos1);

            // Incrementally adjust wristServo2 position
            if (currentPos2 < WRIST_2_PICKUP) {
                currentPos2 = Math.min(currentPos2 + POSITION_INCREMENT, WRIST_2_PICKUP);
            } else if (currentPos2 > WRIST_2_PICKUP) {
                currentPos2 = Math.max(currentPos2 - POSITION_INCREMENT, WRIST_2_PICKUP);
            }
            wristServo2.setPosition(currentPos2);

            // Optional: Add a short delay for smoother movement (e.g., 20ms)
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public void setWristDefaultPosition() {
        wristServo1.setPosition(WRIST_1_DEFAULT);
        wristServo2.setPosition(WRIST_2_DEFAULT);
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

    }
}
