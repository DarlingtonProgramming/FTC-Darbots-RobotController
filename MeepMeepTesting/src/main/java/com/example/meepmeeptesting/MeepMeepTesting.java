package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d start = new Pose2d(30, 63, toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(13, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .forward(2)
                                .strafeRight(20)
                                //.forward(50)
                                //.turn(toRadians(43))
                                .lineToSplineHeading(new Pose2d(16, 8, toRadians(315)))
                                .waitSeconds(2)
                                .back(5)
                                .lineToSplineHeading(new Pose2d(62, 11.5, toRadians(0)))
                                .lineToSplineHeading(new Pose2d(23.5, 12, toRadians(270)))
                                .waitSeconds(2)
                                .forward(5)
                                .waitSeconds(1)
                                .back(5)
                                .strafeLeft(15)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}