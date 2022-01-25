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

@Autonomous(name = "LevelGoBrrr",group = "Auto")
public class HeeHeeLevelSelect extends LinearOpMode {
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
        robot.initializeRobot(hardwareMap, telemetry, RobotDrive.allianceColor.red);


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
        robot.turnOnLights();
        waitForStart();
        while (opModeIsActive()) {
            switch (detector.getLocation()) {
                case LEFT:
                    barcode = 2;
                    break;
                case RIGHT:
                    barcode = 0;
                    break;
                case MIDDLE:
                    barcode = 1;
                    break;
                default:
                    barcode = -1;
                    break;
            }

            if (barcode > -1)
                robot.nextLevel(robot.levels[barcode]);
            Thread.sleep(2000);
        }
        webcam.stopStreaming();

        telemetry.addData("Detected Level: ", barcode);
        telemetry.update();


    }
}
