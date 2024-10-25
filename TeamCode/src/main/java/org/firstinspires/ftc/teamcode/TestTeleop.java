package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Teleop", group = "Teleop")
public class TestTeleop extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);

        while (opModeIsActive()) {

            if (gamepad1.a) {
                robot.intake.startIntake(gamepad1);
            } else {
                robot.intake.stopIntake();
            }


            robot.update();
        }
    }
}
