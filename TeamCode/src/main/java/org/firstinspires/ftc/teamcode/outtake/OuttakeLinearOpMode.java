package org.firstinspires.ftc.teamcode.outtake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

@TeleOp(name = "Outtake OpMode")
class OuttakeOpMode extends OpMode {
    private OuttakeSubsystem outtakeSubsystem;

    @Override
    public void init() {
        // Initialize the outtake subsystem
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        // Control the outtake with gamepad buttons
        if (gamepad1.right_bumper) {
            outtakeSubsystem.runOuttake();  // Run outtake when right bumper is pressed
        } else {
            outtakeSubsystem.stopOuttake();  // Stop outtake when released
        }
    }
}
