package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
    public static DriveMethod.poseState state = DriveMethod.poseState.UNKNOWN;
    public static DriveMethod.poseState autoState = DriveMethod.poseState.UNKNOWN;
}
