package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class BlueAlianceNear extends LinearOpMode {
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
        robot.initializeRobot(hardwareMap,telemetry, RobotDrive.allianceColor.blue);


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
        TrajectorySequence mainDrive = drive.trajectorySequenceBuilder(new Pose2d(-36, 70 - 15/2, Math.toRadians(-90)))
                .forward(15/2)
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-36,23))
                .forward(2*2)
                .addDisplacementMarker(() -> {
                    //claw move and drop block
                })
                .waitSeconds(0.5)
                .back(1)
                .strafeTo(new Vector2d(-55,55))
                .addDisplacementMarker(() -> {
                    // add duckwheel moving
                })
                .waitSeconds(3)
                .forward(1)
                .splineTo(new Vector2d(-60,36),Math.toRadians(-90))
                .build();

        robot.turnOnLights();

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

    }
}
