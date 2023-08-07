package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Striker.TeleOp;


public class StrikerConstants {
    public static final boolean IS_COMPETITION = false;

    // Enums
    public enum ALLIANCE {
        BLUE,
        RED,
        COUCH
    }

    // Speed
    public static double INITIAL_SPEED = 0.6;
    public static double LOW_SPEED = 0.3;
    public static double HIGH_SPEED = 0.6;
    public static double SPEED_INCREMENT = 0.1;

    // Turn
    public static int TURN_INITIAL = 290;
    public static double TURN_90 = 145;

    // Slide
    public static int SLIDE_SMALL = 1845;
    public static int SLIDE_MEDIUM = 3640;
    public static int SLIDE_HIGH = 5600;
    public static int SLIDE_INCREMENT = 250;

    // Pinch
    public static double PINCH_CLOSED = 1;
    public static double PINCH_OPEN = 0.7;
    public static double PINCH_THRESHOLD = (PINCH_CLOSED + PINCH_OPEN) / 2;
}
