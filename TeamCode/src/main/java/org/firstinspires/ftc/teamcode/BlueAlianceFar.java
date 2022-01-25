package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueAlianceFar")
public class BlueAlianceFar extends LinearOpMode {
    OpenCvCamera webcam;
    int barcode;
    @Override
    public void runOpMode() throws InterruptedException {
        //initalizing the webcam for OPENCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //create an object to detect the Team shipping element
        OpenCV detector = new OpenCV(telemetry);

        //create a RobotDrive Object and initalize it
        RobotDrive robot = new RobotDrive();
        robot.initializeRobot(hardwareMap,telemetry, RobotDrive.allianceColor.red);


        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence mainDrive = drive.trajectorySequenceBuilder(new Pose2d(10, 70 - 15/2, Math.toRadians(-90)))
                .forward(15/2)
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(10,24))
                .forward(barcode * 1)
                .addDisplacementMarker(() -> {
                    robot.Grabber.setPosition(1);
                })
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(5,8))
                .forward(20)
                .splineTo(new Vector2d(-45,20),Math.toRadians(90))
                .back(11)
                .strafeTo(new Vector2d(-55,55))
                .addDisplacementMarker(() -> {
                    robot.duckMotor.setPower(1);
                })
                .waitSeconds(3)
                .addDisplacementMarker(() -> {
                    robot.duckMotor.setPower(0);
                })
                .forward(1)
                .splineTo(new Vector2d(-55,20),Math.toRadians(0))
                .strafeTo(new Vector2d(10,8))
                .strafeTo(new Vector2d(10,40))
                .turn(Math.toRadians(-90))
                .build();

        waitForStart();

        switch (detector.getLocation())
        {
            case LEFT:
                barcode = 2;
                break;
            case RIGHT:
                barcode= 0;
                break;
            case MIDDLE:
                barcode = 1;
                break;
            default:
                barcode = -1;
                break;
        }

        webcam.stopStreaming();

        telemetry.addData("Detected Level: ",barcode);
        telemetry.update();
        robot.turnOnLights();
        if(barcode > -1)
            robot.nextLevel(robot.levels[barcode]);

        drive.followTrajectorySequence(mainDrive);
        robot.liftOdoPods();
        robot.mixDrive(1,0,0);
        Thread.sleep(400);
        robot.mixDrive(0,0,0);

    }
}
