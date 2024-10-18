package org.firstinspires.ftc.teamcode.intake.intake.intake.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.outtake.OuttakeCommand;

@TeleOp(name = "Intake Control LinearOpMode", group = "LinearOpMode")
public class IntakeLinearOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        IntakeSubsystem intakeSubsystem;
        OuttakeCommand intakeCommand;

        // Initialize the subsystem
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);

        // Wait for the game to start
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (until stop is pressed)
        while (opModeIsActive()) {
            // Control the intake with gamepad buttons
            if (gamepad1.dpad_down) {
                intakeCommand = new OuttakeCommand(intakeSubsystem, OuttakeCommand.OuttakeState.START, telemetry);
                intakeCommand.execute();
            } else if (gamepad1.dpad_up) {
                intakeCommand = new OuttakeCommand(intakeSubsystem, OuttakeCommand.OuttakeState.STOP, telemetry);
                intakeCommand.execute();
            } else if (gamepad1.dpad_right) {
                intakeCommand = new OuttakeCommand(intakeSubsystem, OuttakeCommand.OuttakeState.REVERSE, telemetry);
                intakeCommand.execute();
            }

            // Small delay to prevent overloading the CPU
            sleep(100);
        }
    }
}
