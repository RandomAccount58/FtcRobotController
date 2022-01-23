package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static androidx.core.math.MathUtils.clamp;

public class RobotDrive {

    Telemetry telemetry = null;
    allianceColor teamColor = null;
    //set levels for the bottom, mid, and high tower
    public final int levels[] = {-500,-1800,-3400};
    public MultiThread autoLevel = new MultiThread("Auto Height", this );


    //Hardware
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public DcMotorEx[] motors = new DcMotorEx[4];
    private BNO055IMU imu = null;
    public DistanceSensor dist = null;
    public ColorSensor colorSensor = null;
    public RevBlinkinLedDriver lights = null;

    //Accessory motors/devices like intake and chainlift
    public DcMotorEx liftMotor;
    public DcMotorEx duckMotor;
    public Servo Grabber; //servo that grabs onto the blocks
    public Servo dropArm;

    //setup the servos for pod raising
    public Servo leftPod;
    public Servo rightPod;
    public Servo frontPod;
    public Servo[] podServos = new Servo[3];


    //Default motor power levels for wheels
    public double motorPower = 1;

    //Debug the error angle in order to get this value, sets the offset to which the robot will turn to meet the required degrees turned
    private final double TURNING_BUFFER = 0;

    enum allianceColor {
        red, blue
    }


    //Assigning software objects to hardware, receives hardwareMap and telemetry objects from the op mode which calls it
    void initializeRobot(HardwareMap hardwareMap, Telemetry telem, allianceColor clr) {
        telemetry = telem;
        teamColor = clr;

        //Initialize hardware from hardware map
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("front_left_motor");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("front_right_motor");
        leftRear = (DcMotorEx) hardwareMap.dcMotor.get("back_left_motor");
        rightRear = (DcMotorEx) hardwareMap.dcMotor.get("back_right_motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        dist = hardwareMap.get(Rev2mDistanceSensor.class, "lift_Distance");
//        colorSensor = hardwareMap.get(ColorSensor.class, "floor_color");

//        //Initalize accessory hardware from hardware map
        liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("lift_motor");
        duckMotor = (DcMotorEx) hardwareMap.dcMotor.get("duck_motor");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        dropArm = hardwareMap.servo.get("drop_arm");
        Grabber = (Servo)hardwareMap.servo.get("block_claw");

        //Initalize the servos for Odometry Pod Lift
        leftPod = hardwareMap.servo.get("left_pod_servo");
        rightPod = hardwareMap.servo.get("right_pod_servo");
        frontPod = hardwareMap.servo.get("front_pod_servo");
        podServos[0] = leftPod;
        podServos[1] = rightPod;
        podServos[2] = frontPod;


        //Sensor Initialization
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(false);
        }

        //Motor initialization
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        //rightRear.setDirection(DcMotor.Direction.REVERSE);

        //Initalize the accessory devices
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        motors[0] = leftFront;
        motors[1] = leftRear;
        motors[2] = rightFront;
        motors[3] = rightRear;
    }

        //Driving function that accepts three values for forward, strafe, and rotate, then mixes those values into 4 usable motor power outputs and sends to the motors.
    void mixDrive(double forward, double strafe, double rotate) {
        double frontLeftSpeed = clamp((forward + strafe + rotate), -motorPower, motorPower);
        double frontRightSpeed = clamp((forward - strafe - rotate), -motorPower, motorPower);
        double backLeftSpeed = clamp((forward - strafe + rotate ), -motorPower, motorPower);
        double backRightSpeed = clamp((forward + strafe - rotate), -motorPower, motorPower);

        leftFront.setPower(frontLeftSpeed);
        rightFront.setPower(frontRightSpeed);
        leftRear.setPower(backLeftSpeed);
        rightRear.setPower(backRightSpeed);
    }

    //moves the lift down until the distance sensor detects that it is at the bottom and reset the motor endoder so it is correct
    void initLift() {
        while (dist.getDistance(DistanceUnit.INCH) > 2) {
            liftMotor.setPower(-1);
        }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void nextLevel(int targetHeight) {
        autoLevel.terminate();
        autoLevel.changeLevel(targetHeight);
        autoLevel.start();
    }

    //cancel any auto Level
    void moveLiftManual(float leftTrigger,float rightTrigger) {
        //check to see if they are pressing on the triggers and cancel the auto level
        if(leftTrigger > 0.05 || rightTrigger > 0.05) {
            autoLevel.terminate();
        }

        //move the lift with the desired player input if they are too close to the limits dont move
            if (liftMotor.getCurrentPosition() <= -4400)
                liftMotor.setPower(rightTrigger);
            else if (dist.getDistance(DistanceUnit.INCH) <= 2)
                liftMotor.setPower(-leftTrigger);
            else if ((!(liftMotor.getCurrentPosition() <= -4400) || (dist.getDistance(DistanceUnit.INCH) <= 2)))
                liftMotor.setPower(rightTrigger - leftTrigger);
    }

    void liftOdoPods() {
        for (Servo pod:podServos) {
            pod.setPosition(1);
        }
    }

    void turnOnLights()
    {
        if(teamColor == allianceColor.blue)
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

    }
}
