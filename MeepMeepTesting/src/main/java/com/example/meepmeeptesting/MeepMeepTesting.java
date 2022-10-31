package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d start = //new Pose2d(28.5, 63, toRadians(270));
            new Pose2d(31, 64, toRadians(270));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setConstraints(46.743540471846245, 46.743540471846245, 15.851832582393046, 4.373484018011914, 9.7)
                .setDimensions(12.25, 15.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .strafeRight(22)
                                .lineToSplineHeading(new Pose2d(16, 8, toRadians(315)))
                                .back(9)
                                .lineToSplineHeading(new Pose2d(64, 12, toRadians(0)))                                .waitSeconds(0.75)
                                .lineToSplineHeading(new Pose2d(47.5, 13, toRadians(90)))
                                /*
                                .back(7)
                                .lineToSplineHeading(new Pose2d(58.5, 12, toRadians(0)))
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(23.5, 12, toRadians(270)))
                                .forward(3)
                                .waitSeconds(1.5)
                                .back(3)

                                .lineToSplineHeading(new Pose2d(58.5, 12, toRadians(0)))
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(23.5, 12, toRadians(270)))
                                .forward(3)
                                .waitSeconds(1.5)
                                .back(3)

                                .lineToSplineHeading(new Pose2d(58.5, 12, toRadians(0)))
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(23.5, 12, toRadians(270)))
                                .forward(3)
                                .waitSeconds(1.5)
                                .back(3)

                                .strafeLeft(35)*/
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}