package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumDriveCommand {
    private final MecanumDriveCommand drivetrain;
    private Gamepad gamepad = new Gamepad();

    public MecanumDriveCommand() {
        this.drivetrain = new MecanumDriveCommand();
        MecanumDriveCommand drivetrain1 = drivetrain;
        this.gamepad = gamepad;
    }

    // This method will be called continuously in the OpMode loop
    public void execute() {
        // Get joystick values for movement
        double y = -gamepad.left_stick_y;  // Forward/backward
        double x = gamepad.left_stick_x;   // Left/right (strafing)
        double rotate = gamepad.right_stick_x;  // Rotation

        // Control the drivetrain based on joystick input
        drivetrain.drive(y, x, rotate);
    }

    private void drive(double y, double x, double rotate) {
    }

    // Method to stop the drivetrain
    public void stop() {
        drivetrain.stop();
    }
}
