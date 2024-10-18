package org.firstinspires.ftc.teamcode.intake.intake.intake.intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeCommand {

    private IntakeSubsystem intakeSubsystem;
    private Telemetry telemetry;

    // Enum for Intake control state
    public enum IntakeState {
        START, STOP, REVERSE
    }

    private IntakeState intakeState;

    // Constructor to set up the command with the desired intake state
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeState intakeState, Telemetry telemetry) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeState = intakeState;
        this.telemetry = telemetry;
    }

    // Execute the command based on the state
    public void execute() {
        switch (intakeState) {
            case START:
                intakeSubsystem.startIntake();
                telemetry.addData("Command", "Intake Started");
                break;

            case STOP:
                intakeSubsystem.stop();
                telemetry.addData("Command", "Intake Stopped");
                break;

            case REVERSE:
                intakeSubsystem.reverseIntake();
                telemetry.addData("Command", "Intake Reversed");
                break;
        }
        telemetry.update();
    }
}
