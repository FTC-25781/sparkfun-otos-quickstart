package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="ViperSlideLimitSwitch", group="TeleOp")
public class ViperSlideLimitSwitch extends OpMode {
    private DcMotor slideMotor;          // Motor controlling the Viper slides
    private DigitalChannel limitSwitch;   // Limit switch to stop slide movement

    @Override
    public void init() {
        // Initialize the motor and limit switch
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        // Set the limit switch to INPUT mode
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        double slidePower = -gamepad1.left_stick_y;  // Control slide with left joystick (up/down)

        // Check if the limit switch is pressed
        if (!limitSwitch.getState()) {  // If the switch is pressed, stop slide motor
            slideMotor.setPower(0);
            telemetry.addData("Limit Switch", "Pressed - Slide Motor Stopped");
        } else {
            slideMotor.setPower(slidePower); // Allow normal control if switch isn't pressed
            telemetry.addData("Limit Switch", "Not Pressed - Slide Motor Moving");
        }

        // Display the status on the driver station
        telemetry.addData("Slide Power", slidePower);
        telemetry.update();
    }

    @Override
    public void stop() {
        slideMotor.setPower(0);  // Stop the motor when the OpMode is stopped
    }
}
