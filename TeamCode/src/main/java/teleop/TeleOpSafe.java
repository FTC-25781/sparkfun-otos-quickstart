package teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.outoftheboxrobotics.photoncore.Photon;
//import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.RobotFlags;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.concurrent.TimeUnit;

//@Photon
@TeleOp(group = "competition")
public class TeleOpSafe extends LinearOpMode {

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,telemetry);

        double x;
        double y;
        double rx;

//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(4);

        ElapsedTime matchTimer;

        int buzzers = 0;
        double loopTime = 0.0;
        long sumLoop = 0;
        long loopIterations = 0;

        boolean previousDpadLeftState = false;
        boolean previousDpadRightState = false;
        boolean previousDroneState = false;
        boolean previousIntakeState = false;
        boolean previousDpadUp = false;
        float previousLeftTriggerState = 0;
        boolean previousSquare = false;
        boolean previousTriangle = false;
        boolean previousCircle = false;

        boolean[] detectedIndex;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        matchTimer = new ElapsedTime();
        //intakePreviousPos = robot.intake.intakeMotor.getCurrentPosition();
//        robot.slides.runToPosition(0);
//        robot.slides.resetAllEncoders();
//        robot.drone.prime();

        while (opModeIsActive() && !isStopRequested()) {
//
//            if (gamepad1.dpad_up) {
//                robot.slides.resetAllEncoders();
//            }

            //DRIVE
            if (gamepad1.right_trigger > 0.5) {
                x = -gamepad1.left_stick_x * (1 - 0.66 * gamepad1.right_trigger);
                y = -gamepad1.left_stick_y * (1 - 0.66 * gamepad1.right_trigger);
                rx = gamepad1.right_stick_x * (1 - 0.66 * gamepad1.right_trigger);

            } else {
                x = -gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                rx = gamepad1.right_stick_x;
            }
            robot.setDrivePower(-x, y, rx);

            previousLeftTriggerState = gamepad1.left_trigger;

            previousDpadLeftState = gamepad1.dpad_left;
            previousDpadRightState = gamepad1.dpad_right;
        }}}


