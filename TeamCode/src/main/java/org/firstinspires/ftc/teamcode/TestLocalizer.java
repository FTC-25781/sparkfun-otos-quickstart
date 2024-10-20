//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.DualNum;
//import com.acmerobotics.roadrunner.Time;
//import com.acmerobotics.roadrunner.Twist2dDual;
//import com.acmerobotics.roadrunner.Vector2dDual;
//import com.acmerobotics.roadrunner.ftc.FlightRecorder;
//import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class TestLocalizer implements Localizer {
//
//    GoBildaPinpointDriverRR odo;
//
//    public TestLocalizer(HardwareMap hardwareMap) {
//        odo = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
//
//        /*
//        Set the odometry pod positions relative to the point that the odometry computer tracks around.
//        The X pod offset refers to how far sideways from the tracking point the
//        X (forward) odometry pod is. Left of the center is a positive number,
//        right of center is a negative number. the Y pod offset refers to how far forwards from
//        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
//        backwards is a negative number.
//         */
//        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
//
//        /*
//        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
//        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
//        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
//        number of ticks per mm of your odometry pod.
//         */
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
//        //odo.setEncoderResolution(13.26291192);
//
//
//        /*
//        Set the direction that each of the two odometry pods count. The X (forward) pod should
//        increase when you move the robot forward. And the Y (strafe) pod should increase when
//        you move the robot to the left.
//         */
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
//
//        /*
//        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
//        The IMU will automatically calibrate when first powered on, but recalibrating before running
//        the robot is a good idea to ensure that the calibration is "good".
//        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
//        This is recommended before you run your autonomous, as a bad initial calibration can cause
//        an incorrect starting value for x, y, and heading.
//         */
//        //odo.recalibrateIMU();
//        odo.resetPosAndIMU();
//
//        //FlightRecorder.write("PINPOINT_PARAMS", PARAMS);
//    }
//    @Override
//    public Twist2dDual<Time> update() {
//
//
//        /**
//         * Twist2dDual: The Twist2dDual object represents the robot's velocity in both linear (x, y)
//         * and angular (heading) directions. It also includes the rate of change of these velocities.
//         */
//        Twist2dDual<Time> twist = new Twist2dDual<>(
//                new Vector2dDual<>(
//                        new DualNum<Time>(new double[] {
//                                parPosDelta - PARAMS.parYTicks * headingDelta,
//                                parVel - PARAMS.parYTicks * headingVel,
//                        }).times(1),
//                        new DualNum<Time>(new double[] {
//                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
//                                perpVel - PARAMS.perpXTicks * headingVel,
//                        }).times(1)
//                ),
//                new DualNum<>(new double[] {
//                        headingDelta,
//                        headingVel,
//                })
//        );
//
//        lastParPos = parPos;
//        lastPerpPos = perpPos;
//        lastHeading = heading;
//
//        return twist;
//    }
//}
