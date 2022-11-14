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
        RoadRunnerBotEntity redRight = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setConstraints(46.743540471846245, 46.743540471846245, 15.851832582393046, 4.373484018011914, 9.7)
                .setDimensions(12.25, 15.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40.3, 62, toRadians(270)))
                                .waitSeconds(4)
                                .lineToSplineHeading(new Pose2d(-33, 55.5, toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-31, -3, toRadians(15)))
                                .lineToSplineHeading(new Pose2d(-35, 12, toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-60, 12, toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-35, 12, toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-29.5, 7.5, toRadians(315)))
                                .build()
                );
        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setConstraints(46.743540471846245, 46.743540471846245, 15.851832582393046, 4.373484018011914, 9.7)
                .setDimensions(12.25, 15.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(64, 12, toRadians(0)))
                                .lineToSplineHeading(new Pose2d(23.5, 12, toRadians(270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redRight)
                .addEntity(blueLeft)
                .setAxesInterval(12)
                .start();
    }
}