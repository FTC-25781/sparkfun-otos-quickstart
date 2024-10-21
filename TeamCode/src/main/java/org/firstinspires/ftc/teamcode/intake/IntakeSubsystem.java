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

    // Constraints
    private final double slide_out_motor = 1.0;
    private final double wrist_start_both = 0.5;
    private final double servo1_pick = 0.0;
    private final double servo2_pick = 1.0;
    private final double claw_open = 1.0;
    private final double servo1_up = 1.0;
    private final double servo2_up = 0.0;
    private final double slide_in_motor = 0.0;

    // Variables
    double final_pos_motor = 800;
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

        // Initialize VisionAngleSub (make sure the VisionAngleSub pipeline is working)
        servoOrientation = new VisionAngleSub(); // Assuming this is correct initialization
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
            wristServo1.setPosition(wrist_start_both); // Neutral position
            wristServo2.setPosition(wrist_start_both);

            // 3. Get and use the orientation value
            double orientation = servoOrientation.getOrientation(); // Fetch the orientation from VisionAngleSub
            telemetry.addData("Claw Orientation", orientation);
            telemetry.update();

            // Use the orientation value for whatever action you want (like setting servo positions)
            orientationServo.setPosition(orientation); // Adjust servo based on orientation
        }
    }

    // Method to stop the intake
    public void stopIntake() {
        // Add stop logic here
    }
}
