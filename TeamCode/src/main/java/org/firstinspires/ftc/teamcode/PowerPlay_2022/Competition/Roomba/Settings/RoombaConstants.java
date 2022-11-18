package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings;

import static java.lang.Math.toRadians;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class RoombaConstants {
    public enum AllianceType {
        BLUE,
        RED
    }
    // Recording
    public static final int OCV_RECORDING_WIDTH = 960;
    public static final int OCV_RECORDING_HEIGHT = 720;

    // Vision
     /* OpenCV */
     public static final int OCV_STREAMING_WIDTH = 960;
     public static final int OCV_STREAMING_HEIGHT = 720;

     /* Custom TF Model */
     public static final String BLUE_MODEL = "Blue.tflite";
     public static final String RED_MODEL = "Red.tflite";
     public static final String LABEL_FILE_NAME = "labels.txt";
     public static final String[] LABELS = {
            "0 eyes",
            "1 bat",
            "2 lantern"
     };
     public static final double CONFIDENCE_LEVEL = 0.5f;

    // Auto Constants
    public static final int[] CONE_HEIGHTS = { 425, 280 };
    public static final Pose2d BLUE_LEFT_STARTING = new Pose2d(30, 61, toRadians(270));
    public static final Pose2d BLUE_LEFT_INITIAL_STRAFE = new Pose2d(36, 55.5, toRadians(180));
    public static final Pose2d BLUE_LEFT_END_STRAFE = new Pose2d(37.5, 0, toRadians(180));
    public static final Pose2d BLUE_LEFT_MIDPOINT = new Pose2d(38, 12, toRadians(0));
    public static final Pose2d BLUE_LEFT_CONE = new Pose2d(63, 12, toRadians(0));
    public static final Pose2d BLUE_LEFT_HIGH_JUNC = new Pose2d(32, 7, toRadians(253));

    public static final Pose2d RED_RIGHT_STARTING = new Pose2d(-40, 62, toRadians(270));
    public static final Pose2d RED_RIGHT_INITIAL_STRAFE = new Pose2d(-32, 55.5, toRadians(0));
    public static final Pose2d RED_RIGHT_END_STRAFE = new Pose2d(-29, -1, toRadians(0));
    public static final Pose2d RED_RIGHT_MIDPOINT = new Pose2d(-38, 10, toRadians(180));
    public static final Pose2d RED_RIGHT_CONE = new Pose2d(-58, 9, toRadians(180));
    public static final Pose2d RED_RIGHT_HIGH_JUNC = new Pose2d(-28, 4, toRadians(320));

    // Drive Constants
    public static final double INITIAL_SPEED = 0.5;
    public static final double LOW_SPEED = 0.35;
    public static final double HIGH_SPEED = 0.55;
    public static final double SPEED_INCREMENT = 0.1;

    // Components Constants
    public static final double PINCH_MIN = 0.03;
    public static final double PINCH_MAX = 0.8;
    public static final double PINCH_MIDPOINT = (PINCH_MAX + PINCH_MIN) / 2;

    public static final int SL_LOW = 1200;
    public static final int SL_MEDIUM = 2000;
    public static final int SL_HIGH = 2800;
}