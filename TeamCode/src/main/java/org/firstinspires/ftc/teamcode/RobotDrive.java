package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

    //Hardware
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotorEx[] motors = new DcMotorEx[4];
    private BNO055IMU imu = null;
    public ColorSensor dist = null;
    public ColorSensor colorSensor = null;

    //Accessory motors/devices like intake and chainlift
    public DcMotorEx liftMotor;
//    public DcMotorEx chainLift;
//    public Servo dropArm; //servo that drops the intake wheels
//    public DcMotorEx flyWheel;
//    public DcMotorEx wobbleArm;
//    public Servo wobbleClaw;
//    public DigitalChannel armTrigger;

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
        dist = hardwareMap.get(ColorSensor.class, "lift_Distance");
//        colorSensor = hardwareMap.get(ColorSensor.class, "floor_color");

//        //Initalize accessory hardware from hardware map
        liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("lift_motor");
//        chainLift = (DcMotorEx)hardwareMap.dcMotor.get("chain_motor");
//        dropArm = (Servo)hardwareMap.servo.get("drop_arm");
//        flyWheel = (DcMotorEx)hardwareMap.dcMotor.get("flywheel_motor");
//        wobbleArm = (DcMotorEx)hardwareMap.dcMotor.get("wobble_arm");
//        wobbleClaw = (Servo)hardwareMap.servo.get("wobble_servo");
//        armTrigger = hardwareMap.get(DigitalChannel.class, "Arm_Button");
//        armTrigger.setMode(DigitalChannel.Mode.INPUT);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
}
