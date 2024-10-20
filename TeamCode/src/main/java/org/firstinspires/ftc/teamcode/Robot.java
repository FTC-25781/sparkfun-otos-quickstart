package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Component;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.MotorEx;
import org.firstinspires.ftc.teamcode.lib.RobotFlags;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
//import org.firstinspires.ftc.teamcode.subsystems.arm.ArmElbow;
//import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSensor;
//import org.firstinspires.ftc.teamcode.subsystems.relocalization.Relocalization;
//import org.firstinspires.ftc.teamcode.subsystems.slides.Slides;
//import org.firstinspires.ftc.teamcode.subsystems.launcher.DroneLauncher;
//import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {

    private final Telemetry telemetry;
    private final IntakeSubsystem intake;
    // SUBSYSTEM DECLARATIONS
    public Component[] components;

//    public Intake intake;
    ;
    public HardwareMap hardwareMap;

    // STATE VARS
    boolean auton;
    public Levels subsystemState = Levels.INTAKE;
    public boolean intaking = false;
    public ArrayList<RobotFlags> flags = new ArrayList<>();
    public double CURRENT_HIGH = 1;
    public double ENCODER_MAX_DIFFERENCE = 1;
    public ElapsedTime antiJamCooldown = new ElapsedTime();
    public boolean threadState = false;

    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;


    public Robot(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;



//        this.cv = new CVMaster(map);
//        this.components = new Component[]{
//                new Motor(3, "leftRear", map, true),          //0 left odometer
//                new Motor(2, "rightRear", map, false),        //1 right odometer
//                new Motor(1, "leftFront", map, true),         //2 middle odometer
//                new Motor(0, "rightFront", map, false),       //3
//
//                new Motor(0, "slides1", map, false),          //4
//                new Motor(0, "slides2", map, false),          //5
//                new Motor(0, "climb", map, false),            //6
//                new StepperServo(1, "shifter", map),                 //7
//
//                new StepperServo(1, "arm1", map),                    //8
//                new StepperServo(1, "arm2", map),                    //9
//                new StepperServo(1, "elbow", map),                   //10
//                new StepperServo(1, "claw", map),                 //11
//                new StepperServo(1, "wrist", map),                   //12
//                new MotorEx(1, "intakeMotor", map, false),   //13
//                new StepperServo(1, "intakeServo1", map),            //14
//                new StepperServo(1, "intakeServo2", map),            //15
//
//                new StepperServo(1, "drone", map),                   //16
//
//        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();

        // INIT SUBSYSTEMS


        this.intake = new IntakeSubsystem(hardwareMap, telemetry);

        this.hardwareMap = map;

        this.subsystemState = Levels.ZERO;


        //FtcDashboard.getInstance().getTelemetry().speak("Hello Mr. Juice");
    }

    // INTAKE
    public void intakePreset() {
        //turning to get through the thingy

//        this.intake.runToPreset(Levels.INTAKE);

    }

    public void startIntake() {
        intaking = true;

//        this.intake.runToPreset(Levels.INTAKE);

    }

    public void startAutoIntake() {
        intaking = true;

//        this.intake.runToPreset(Levels.INTAKE);

    }

//    public void stopIntake() {
//        intaking = false;
//        this.arm.runtoPreset(Levels.CAPTURE);
//        Thread thread = new Thread(new Runnable() {
//            public void run() {
//
//                intake.stopIntake();
//                intake.runToPreset(Levels.INTERMEDIATE);
//
//            }});
//        thread.start();
//        subsystemState = Levels.INTERMEDIATE;
//    }


    //DRIVE
    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }

        frontLeft.setSpeed((float)powerFrontLeft);
        frontRight.setSpeed((float)powerFrontRight);
        backLeft.setSpeed(-(float)powerBackLeft);
        backRight.setSpeed(-(float)powerBackRight);
    }

    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
