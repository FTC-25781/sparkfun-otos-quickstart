//package org.firstinspires.ftc.teamcode.tuning;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.PinpointDrive;
//
//public final class SplineTest extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d beginPose = new Pose2d(0, 0, 0);
//        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
//            PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
//
//            waitForStart();
//
//            Actions.runBlocking(
//                    drive.actionBuilder(beginPose)
//                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                            .splineTo(new Vector2d(0, 60), Math.PI)
//                            .build());
//
//            telemetry.addData("Executed spline one", "Spline to (30, 30) executed");
//            telemetry.addData("Executed spline two", "Spline to (0, 60) executed");
//            telemetry.update();
//            sleep(5000);
//
//        } else {
//            throw new RuntimeException();
//        }
//    }
//}
