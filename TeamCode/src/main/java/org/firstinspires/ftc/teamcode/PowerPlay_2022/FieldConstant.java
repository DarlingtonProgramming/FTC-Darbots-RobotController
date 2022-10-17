package org.firstinspires.ftc.teamcode.PowerPlay_2022;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import static java.lang.Math.toRadians;

public class FieldConstant {
    public static final Pose2d BLUE_TOP = new Pose2d();
    public static final Pose2d BLUE_BOTTOM = new Pose2d();

    public static final Pose2d RED_TOP = new Pose2d();
    public static final Pose2d RED_BOTTOM = new Pose2d(62, -34, toRadians(180));

    public static final Pose2d BB_BOLT_PARK = new Pose2d();
    public static final Pose2d BB_BULB_PARK = new Pose2d();
    public static final Pose2d BB_PANEL_PARK = new Pose2d();

    public static final Pose2d RB_BOLT_PARK = new Pose2d(35, -60, toRadians(90));
    public static final Pose2d RB_BULB_PARK = new Pose2d(35, -34, toRadians(90));
    public static final Pose2d RB_PANEL_PARK = new Pose2d(35, -13, toRadians(90));
}
