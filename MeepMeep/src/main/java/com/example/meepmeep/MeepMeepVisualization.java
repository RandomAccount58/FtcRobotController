package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualization {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -70 + 15/2, Math.toRadians(90)))
                                .forward(15/2)
                                .turn(Math.toRadians(-90))
                                .strafeTo(new Vector2d(-36,-23))
                                .forward(2*2)
                                .addDisplacementMarker(() -> {
                                    //robot.Grabber.setPosition(0.75);
                                })
                                .waitSeconds(0.5)
                                .back(1)
                                .splineTo(new Vector2d(-55,-55),Math.toRadians(-90))
                                .addDisplacementMarker(() -> {
                                    //robot.duckMotor.setPower(0.75);
                                })
                                .waitSeconds(3)
                                .forward(1)
                                .splineTo(new Vector2d(-60,-36),Math.toRadians(90))
                                .build()
                );

        /*     *I just comment this in to use when testing*
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -70 + 15/2, Math.toRadians(90)))
                                .forward(15/2)
                                .turn(Math.toRadians(90))
                                .strafeTo(new Vector2d(10,-24))
                                .addDisplacementMarker(() -> {
                                    //claw move and drop block
                                })
                                .waitSeconds(0.5)
                                .strafeTo(new Vector2d(5,-8))
                                .forward(20)
                                .splineTo(new Vector2d(-45,-20),Math.toRadians(0))
                                .back(11)
                                .strafeTo(new Vector2d(-55,-55))
                                .addDisplacementMarker(() -> {
                                    // add duckwheel moving
                                })
                                .waitSeconds(3)
                                .forward(1)
                                .strafeTo(new Vector2d(-55,-20))
                                .splineTo(new Vector2d(10,-8),Math.toRadians(0))
                                .strafeTo(new Vector2d(10,-40))
                                //temporary just do motorpower full for like half a sec
                                .waitSeconds(2)
                                .forward(140)
                                .build()

                );

         */

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}