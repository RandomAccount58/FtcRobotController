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
    private int lvl;
    private boolean lvlWait = false;
    private int liftTolerance = 100;
    RobotDrive robot = new RobotDrive();




    public void runOpMode()
    {
        waitForStart();
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.blue);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set led colors based on what alliance you are on
        if(robot.teamColor == RobotDrive.allianceColor.blue)
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        //drop the arm uppon initalization dont got to do it here just testing do it in auto
        robot.dropArm.setPosition(1);

        while (opModeIsActive()) {


            double forward = gamepad1.left_stick_y * -1; //the gamepad is reversed thus the -1
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x * robot.motorPower;

            robot.mixDrive(forward, strafe, rotate);

            //change the auto level to a diffrent one and check if it has being held down right is up left is down
            if(!gamepad2.left_bumper && !gamepad2.right_bumper)
                lvlWait = true;
            else if(gamepad2.left_bumper && lvl > 0 && lvlWait) {
                lvlWait = false;
                robot.nextLevel(robot.levels[--lvl]);
            }else if(gamepad2.right_bumper && lvl < 2 && lvlWait) {
                lvlWait = false;
                robot.nextLevel(robot.levels[++lvl]);
            }




            //set the led lights for each level
            if(robot.autoLevel.running)
            {
                if(lvl == 0)
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                if(lvl == 1)
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                if(lvl == 2)
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            if(!robot.autoLevel.running)
            {
                if(robot.teamColor == RobotDrive.allianceColor.blue)
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                else
                    robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            //up is left trigger aka moving the motor in the negative direction
            //call the moveLiftManual function in RobotDrive
            robot.moveLiftManual(gamepad2.left_trigger,gamepad2.right_trigger);



            //if the robot arm is at the base position reset arm encoder to ensure accuracy
            if(robot.dist.getDistance(DistanceUnit.INCH) <= 2) {
                robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //slow down the robot if right bumper is pressed
            if(gamepad1.right_bumper)
                robot.motorPower = 0.5;
            else
                robot.motorPower = 1;


            //turn on the duck wheel when gamepad2's a button is pressed
            if(gamepad2.a)
                robot.duckMotor.setPower(-0.75);
            else
                robot.duckMotor.setPower(0);


            //toggle the block grabber arm either on or off
            if(gamepad2.x && robot.Grabber.getPosition() > 0.5 && buttonPressed == false) {
                robot.Grabber.setPosition(0);
                buttonPressed = true;
            } else if(gamepad2.x && robot.Grabber.getPosition() < 0.5 && buttonPressed == false) {
                robot.Grabber.setPosition(0.75);
                buttonPressed = true;
            }else if(!gamepad2.x)
                buttonPressed = false;



            telemetry.addData("auto yn ",robot.autoLevel.running);
            telemetry.addData("liftLvl: ", lvl);
            telemetry.addData("LiftEncoder",robot.liftMotor.getCurrentPosition());
            telemetry.addData("Distance: ","%.3f",((DistanceSensor) robot.dist).getDistance(DistanceUnit.INCH));

            telemetry.update();
        }
        robot.emergencyStop();
    }
}
