package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public enum poseState {
        RED,
        RED_LEFT,
        RED_RIGHT,
        RED_SUBSTATION,
        RED_TERMINAL,
        RED_OTHER,
        BLUE,
        BLUE_LEFT,
        BLUE_RIGHT,
        BLUE_SUBSTATION,
        BLUE_TERMINAL,
        BLUE_OTHER,
        UNKNOWN
    }
    public static Pose2d currentPose = new Pose2d();
    public static poseState state = PoseStorage.poseState.UNKNOWN;
    public static poseState autoState = PoseStorage.poseState.UNKNOWN;
}

