package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "InitalSetup")
public class InitalSetup extends LinearOpMode {

    RobotDrive robot = new RobotDrive();
    public void runOpMode()
    {
        robot.initializeRobot(hardwareMap,telemetry,RobotDrive.allianceColor.blue);
        waitForStart();
        robot.initLift();
        while(!gamepad1.b) {}
        robot.Grabber.setPosition(1);
        while (!gamepad1.a);
    }
}
