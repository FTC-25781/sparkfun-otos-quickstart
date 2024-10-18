package org.firstinspires.ftc.teamcode.outtake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.OuttakeSubsystem;

public class OuttakeCommand {

    private OuttakeSubsystem outtakeSubsystem;
    private Telemetry telemetry;

    // Enum for Intake control state
    public enum OuttakeState {
        START, STOP, REVERSE
    }

    private OuttakeState outtakeState;

    // Constructor to set up the command with the desired intake state
    public OuttakeCommand(OuttakeSubsystem OuttakeSubsystem, OuttakeState outtakeState, Telemetry telemetry) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.outtakeState = outtakeState;
        this.telemetry = telemetry;
    }

    // Execute the command based on the state
    public void execute() {
        switch (outtakeState) {
            case START:
                outtakeSubsystem.runOuttake();
                telemetry.addData("Command", "Intake Started");
                break;

            case STOP:
                outtakeSubsystem.stopOuttake();
                telemetry.addData("Command", "Intake Stopped");
                break;

            case REVERSE:
                outtakeSubsystem.runOuttake();
                telemetry.addData("Command", "Intake Reversed");
                break;
        }
        telemetry.update();
    }
}
