package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d start = new Pose2d(35, 60, toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46.743540471846245, 46.743540471846245, Math.toRadians(250.58217599999998), Math.toRadians(250.58217599999998), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .back(25)
                                .lineToSplineHeading(new Pose2d(10, 34, toRadians(225)))
                                .forward(5)
                                .back(5)
                                .lineToSplineHeading(new Pose2d(12, 12, toRadians(180)))
                                //.forward(50)
                                //.lineToSplineHeading(new Pose2d(-24, -12, toRadians(90)))
                                //.forward(5)
                                //.back(5)
                                .lineToSplineHeading(new Pose2d(0, -60, toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}