package com.example.meepmeeptesting;

import static java.lang.Math.toDegrees;
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
                .setConstraints(123, 123, toRadians(280), toRadians(280), 11.4)
                .setDimensions(12.25, 15.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40.3, 62, toRadians(270)))
                                .waitSeconds(4)
                                .lineToSplineHeading(new Pose2d(36, 55.5, toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-31, -3, toRadians(15)))
                                .lineToSplineHeading(new Pose2d(-35, 12, toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-60, 12, toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-35, 12, toRadians(180)))
                                .splineToLinearHeading(new Pose2d(-29.5, 7.5, toRadians(315)), toRadians(90))
                                .build()
                );
        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setConstraints(60, 55, 6.28, 6.28, 12.7)
                .setDimensions(12.25, 15.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(30, 63.5, toRadians(270)))
                                .lineToSplineHeading(new Pose2d(36, 55.5, toRadians(180)))
                                .lineToSplineHeading(new Pose2d(34, 0, toRadians(180)))
                                .forward(2)
                                .lineToSplineHeading(new Pose2d(37, 11, toRadians(270)))
                                .lineToSplineHeading(new Pose2d(62.5, 11, toRadians(0)))
                                .lineToSplineHeading(new Pose2d(24, 10, toRadians(270)))
                                .forward(3)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueLeft)
                .setAxesInterval(12)
                .start();
    }
}