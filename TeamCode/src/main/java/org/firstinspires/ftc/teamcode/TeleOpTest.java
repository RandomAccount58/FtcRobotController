package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestOp")
public class TeleOpTest extends LinearOpMode{
    private boolean buttonPressed;
    RobotDrive robot = new RobotDrive();
    public void runOpMode()
    {
        waitForStart();
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);

        while (opModeIsActive())
    {


        double forward = gamepad1.left_stick_y * -1; //the gamepad is reversed thus the -1
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * robot.motorPower;

        robot.mixDrive(forward, strafe, rotate);

        if (gamepad1.dpad_up && robot.liftMotor.getCurrentPosition() <= 5870)
            robot.liftMotor.setPower(-1);
        else if(gamepad1.dpad_down && robot.liftMotor.getCurrentPosition() <= 5870)
            robot.liftMotor.setPower(1);
        else if((!gamepad1.dpad_down) && (!gamepad1.dpad_up))
            robot.liftMotor.setPower(0);

        if(gamepad2.x && robot.Grabber.getPosition() > 0.5 && buttonPressed == false) {
            robot.Grabber.setPosition(0);
            buttonPressed = true;
        } else if(gamepad2.x && robot.Grabber.getPosition() < 0.5 && buttonPressed == false) {
            robot.Grabber.setPosition(1);
            buttonPressed = true;
        }else if(!gamepad2.x)
            buttonPressed = false;


        telemetry.addData("Left Rear motor power: ",robot.motors[0].getPower());
        telemetry.addData("GrabberServo: ", robot.Grabber.getPosition());
        telemetry.addData("Distance: ","%.3f",((DistanceSensor) robot.dist).getDistance(DistanceUnit.INCH));
        telemetry.addData("LiftEncoder",robot.liftMotor.getCurrentPosition());
        telemetry.update();
    }
    }
}
