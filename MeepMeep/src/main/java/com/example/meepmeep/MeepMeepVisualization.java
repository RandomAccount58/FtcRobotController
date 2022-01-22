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
                        drive.trajectorySequenceBuilder(new Pose2d(10, -70 + 15/2, Math.toRadians(90)))
                                .splineTo(new Vector2d(-12,-34 - 15/2),Math.toRadians(90))
                                .addDisplacementMarker(() ->
                                {
                                 //add the lift going to level for the barcode
                                })
                                .back(1)
                                .splineTo(new Vector2d(-70 + 15, -70 + 15), Math.toRadians(180))
                                .addDisplacementMarker(() -> {
                                    // add duckwheel moving
                                })
                                .waitSeconds(2)
                                .forward(140)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}