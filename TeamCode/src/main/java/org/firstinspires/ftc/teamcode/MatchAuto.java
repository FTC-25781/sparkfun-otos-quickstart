package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous (name= "MatchAuto", group= "Autonomus");

public class MatchAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMap hardwaremap;
        Robot robot = new Robot(hardwaremap);


    }
}