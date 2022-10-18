package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import static java.lang.Math.toRadians;

public class FieldConstant {
    // Blue
    public static final Pose2d BLUE_LEFT = new Pose2d(35, 60, toRadians(270));
    public static final Pose2d BLUE_RIGHT = new Pose2d(-35, 60, toRadians(270));

    public static final Pose2d BL_BOLT_PARK = new Pose2d(-12, 12, toRadians(90));
    public static final Pose2d BL_BULB_PARK = new Pose2d(-35, 12, toRadians(90));
    public static final Pose2d BL_PANEL_PARK = new Pose2d(-60, 12, toRadians(90));

    public static final Pose2d BR_BOLT_PARK = new Pose2d(60, 12, toRadians(90));
    public static final Pose2d BR_BULB_PARK = new Pose2d(35, 12, toRadians(90));
    public static final Pose2d BR_PANEL_PARK = new Pose2d(12, 12, toRadians(90));

    // Red
    public static final Pose2d RED_LEFT = new Pose2d(-35, -60, toRadians(90));
    public static final Pose2d RED_RIGHT = new Pose2d(35, -60, toRadians(90));

    public static final Pose2d RL_BOLT_PARK = new Pose2d(-60, -12, toRadians(90));
    public static final Pose2d RL_BULB_PARK = new Pose2d(-35, -12, toRadians(90));
    public static final Pose2d RL_PANEL_PARK = new Pose2d(-12, -12, toRadians(90));

    public static final Pose2d RR_BOLT_PARK = new Pose2d(12, -12, toRadians(90));
    public static final Pose2d RR_BULB_PARK = new Pose2d(36, -12, toRadians(90));
    public static final Pose2d RR_PANEL_PARK = new Pose2d(60, -12, toRadians(90));
}
