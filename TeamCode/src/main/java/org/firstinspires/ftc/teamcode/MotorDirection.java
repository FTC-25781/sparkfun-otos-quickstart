package org.firstinspires.ftc.teamcode;// Import the necessary FTC classes

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Motor Control TeleOp", group = "TeleOp")
public class MotorDirection extends OpMode {

    // Declare motor
    private DcMotor motor;

    @Override
    public void init() {
        // Initialize hardware
        motor = hardwareMap.get(DcMotor.class, "motor"); // Replace "motor" with your motor's name in the configuration
    }

    @Override
    public void loop() {
        // Get the left stick's Y axis value
        double leftStickY = gamepad1.left_stick_y;

        // Control motor direction based on left stick
        motor.setPower(leftStickY); // Move forward

        // Update telemetry to display motor power
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.update();
    }
}
