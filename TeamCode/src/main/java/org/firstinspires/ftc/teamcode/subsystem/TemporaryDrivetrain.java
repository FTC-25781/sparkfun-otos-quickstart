package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TemporaryDrivetrain implements Subsystem {

    DcMotor backLeft, backRight, frontLeft, frontRight;

    public TemporaryDrivetrain(HardwareMap hardwareMap) {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

    }

    // add drive function here

    @Override
    public void update() {

    }
}
