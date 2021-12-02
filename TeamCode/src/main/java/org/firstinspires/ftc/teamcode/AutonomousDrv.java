package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOP(name = "TestOp")
public class AutonomousDrv extends LinearOpMode{
    RobotDrive robot = new RobotDrive();
    public void runOpMode()
    {
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);

        double forward = gamepad1.left_stick_y * -1; //the gamepad is reversed thus the -1
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * robot.motorPower;

        robot.mixDrive(forward,strafe,rotate);
    }
}
