package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Roomba_Constants;

public class DriveMethod {
    public enum alliance {
        RED,
        BLUE
    }
    public enum chassis {
        ROOMBA
    }
    private final alliance ALLIANCE;
    private final chassis CHASSIS;
    public DriveMethod(chassis Chassis, alliance Alliance) {
        this.CHASSIS = Chassis;
        this.ALLIANCE = Alliance;
    }
    public static PoseStorage.poseState getCurrentState(Pose2d currentPose) {
        double x = currentPose.getX();
        double y = currentPose.getY();
        if (y < -47.4) {
            if (x < -23.4 && x > -47.1) {
                return PoseStorage.poseState.RED_LEFT;
            }
            if (x > 23.4 && x < 47.1) {
                return PoseStorage.poseState.RED_RIGHT;
            }
            if (x <= -47.1) {
                return PoseStorage.poseState.RED_TERMINAL;
            }
            if (x >= 47.1) {
                return PoseStorage.poseState.BLUE_TERMINAL;
            }
        }
        if (x < 12.7 && x > -12.7) {
            if (y < -58) {
                return PoseStorage.poseState.RED_SUBSTATION;
            }
            if (y > 58)  {
                return PoseStorage.poseState.BLUE_SUBSTATION;
            }
        }
        return PoseStorage.poseState.UNKNOWN;
    }
    public static PoseStorage.poseState getCurrentState(double x, double y) {

        return PoseStorage.poseState.UNKNOWN;
    }
    public static int slideTo(DcMotor Slide, int targetPosition, double power) {
        Slide.setTargetPosition(targetPosition);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);
        return Slide.getCurrentPosition();
    }
    public static double setPinched(Servo Pinch, boolean pinched) {
        if (pinched)
            Pinch.setPosition(Roomba_Constants.PINCH_MAX);
        else
            Pinch.setPosition(Roomba_Constants.PINCH_MIN);
        return Pinch.getPosition();
    }
}
