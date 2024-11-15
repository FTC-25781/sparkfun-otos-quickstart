package org.firstinspires.ftc.teamcode.subsystem.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeV4BSubsystem {
    public Servo wristServo1;
    public Servo wristServo2;

public IntakeV4BSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
    wristServo1 = hardwareMap.get(Servo.class, "wsrv1"); // Servo Port 2
    wristServo2 = hardwareMap.get(Servo.class, "wsrv2"); // Servo Port 3
}


}
