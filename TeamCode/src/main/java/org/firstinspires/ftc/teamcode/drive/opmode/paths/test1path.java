package org.firstinspires.ftc.teamcode.drive.opmode.paths;

// if i think about this enough it will just appear, right?

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class test1path extends LinearOpMode {

//TODO  WAIT DID NOT WORK so between trajectories should wait somehow

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(10, -70 + 15/2, Math.toRadians(90)))
                .forward(15/2)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(10,-24))
                .addDisplacementMarker(() -> {
                    //claw move and drop block
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeTo(new Vector2d(5,-8))
                .forward(20)
                .splineTo(new Vector2d(-45,-20),Math.toRadians(0))
                .back(11)
                .strafeTo(new Vector2d(-55,-55))
                .addDisplacementMarker(() -> {
                    // add duckwheel moving
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(1)
                .strafeTo(new Vector2d(-55,-20))
                .splineTo(new Vector2d(10,-8),Math.toRadians(0))
                .strafeTo(new Vector2d(10,-40))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.turn(Math.toRadians(90));

    }
}
