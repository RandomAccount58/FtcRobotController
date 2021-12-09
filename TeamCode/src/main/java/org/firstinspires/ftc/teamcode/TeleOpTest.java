package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestOp")
public class TeleOpTest extends LinearOpMode{
    RobotDrive robot = new RobotDrive();
    public void runOpMode()
    {
        waitForStart();

        while (opModeIsActive())
    {
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);

        double forward = gamepad1.left_stick_y * -1; //the gamepad is reversed thus the -1
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * robot.motorPower;

        robot.mixDrive(forward, strafe, rotate);

        if (gamepad1.dpad_up)
            robot.liftMotor.setPower(1);
        else if(gamepad1.dpad_down)
            robot.liftMotor.setPower(-1);
        else if((!gamepad1.dpad_down) && (!gamepad1.dpad_up))
            robot.liftMotor.setPower(0);


        telemetry.addData("Distance: ","%.3f",((DistanceSensor) robot.dist).getDistance(DistanceUnit.CM));
        telemetry.addData("UP: ",gamepad1.dpad_up);
        telemetry.addData("DOWN: ",gamepad1.dpad_down);
        telemetry.update();
    }
    }
}
