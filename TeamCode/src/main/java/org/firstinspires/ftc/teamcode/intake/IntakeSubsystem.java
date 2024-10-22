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

        servoOrientation = new VisionAngleSub();
        clawServo.setPosition(0.0);
        orientationServo.setPosition(0.0);
    }

    // Method to start the intake
    public void startIntake(Gamepad gamepad) {
        if (gamepad.a) {
            // 1. Slide extending outward
            slideMotor.setTargetPosition((int) final_pos_motor);
            slideMotor.setPower(motor_extend_speed);
            while (slideMotor.isBusy()) {
                telemetry.addData("Current Position", slideMotor.getCurrentPosition());
                telemetry.update();
            }
            slideMotor.setPower(0);

            // 2. Wrist set position 0.5
            double wrist_start_both = 0.5;
            wristServo1.setPosition(wrist_start_both);
            wristServo2.setPosition(wrist_start_both);

            // 3. Get and use the orientation value
            double orientation = servoOrientation.getOrientation();
            orientationServo.setPosition(orientation);
            telemetry.addData("Claw Orientation", orientation);
            telemetry.update();

            // 4. Open Claw
            double claw_open = 1.0;
            clawServo.setPosition(claw_open);

            // 5. Wrist set down
            double servo1_pick = 0.0;
            wristServo1.setPosition(servo1_pick);
            double servo2_pick = 1.0;
            wristServo2.setPosition(servo2_pick);

            // 6. Close Claw
            clawServo.setPosition(0.0);

            // 7. Wrist up (Going to drop position)
            double servo1_up = 1.0;
            wristServo1.setPosition(servo1_up);
            double servo2_up = 0.0;
            wristServo2.setPosition(servo2_up);

            // 8. Open Claw
            clawServo.setPosition(claw_open);

            // 9. Reset the wrist
            wristServo1.setPosition(wrist_start_both);
            wristServo2.setPosition(wrist_start_both);

            // 10. Bring in slides
            slideMotor.setTargetPosition((int) final_in_pos_motor);
            slideMotor.setPower(-motor_extend_speed);
        }
    }

    // Method to stop the intake
    public void stopIntake() {
        // Add stop logic here
    }
}
