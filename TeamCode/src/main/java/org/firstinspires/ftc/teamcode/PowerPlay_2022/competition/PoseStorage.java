package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    public enum poseState {
        RED,
        RED_TOP,
        RED_BOTTOM,
        BLUE,
        BLUE_TOP,
        BLUE_BOTTOM,
        UNKNOWN
    }
    public enum alliance {
        RED,
        BLUE
    }
    public enum chassis {
        ROOMBA
    }
    public static Pose2d currentPose = new Pose2d();
    public static poseState state = PoseStorage.poseState.UNKNOWN;
    public static poseState autoState = PoseStorage.poseState.UNKNOWN;
}

