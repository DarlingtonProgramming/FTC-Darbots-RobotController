package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Settings.Roomba_Constants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;

public class RoombaDriveMethod {
    private DcMotor Slide;
    private Servo Pinch;
    private MecanumDrive_Roomba drive;

    public RoombaDriveMethod(MecanumDrive_Roomba drive, DcMotor Slide, Servo Pinch) {
        this.Slide = Slide;
        this.Pinch = Pinch;
        this.drive = drive;
    }

    public PoseStorage.poseState getCurrentState() {
        Pose2d currentPose = drive.getPoseEstimate();
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

    public int slideTo(int targetPosition, double power) {
        Slide.setTargetPosition(targetPosition);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);
        return Slide.getCurrentPosition();
    }

    public double setPinched(boolean pinched) {
        if (pinched)
            Pinch.setPosition(Roomba_Constants.PINCH_MAX);
        else
            Pinch.setPosition(Roomba_Constants.PINCH_MIN);
        return Pinch.getPosition();
    }

    public Pose2d stabilizeStrafeDrift(Pose2d initialPose, Pose2d poseTo) {
        double currentX = initialPose.getX();
        double currentY = initialPose.getY();
        double toX = poseTo.getX();
        double toY = poseTo.getY();

        return new Pose2d(toX - ((currentY - toY) / 28.0), toY - ((currentX - toX) / 28.0), poseTo.getHeading());
    }

    public void stackOneMotion(Pose2d stackPose, boolean isLeftSided) {
        double factor = 1;
        if (isLeftSided) factor = -1;
        while (drive.getExternalHeading() > stackPose.getHeading()) {
            drive.setMotorPowers(1*factor, -1*factor, 1*factor, -1*factor);
        }
        stopDrive();
        while (isLeftSided ? drive.getPoseEstimate().getX() < stackPose.getX() : drive.getPoseEstimate().getX() > stackPose.getX()) {
            drive.setMotorPowers(1, 1, 1, 1);
        }
        stopDrive();
    }

    public void junctionOneMotion(Pose2d juncPose, boolean isLeftSided) {
        double factor = 1;
        if (isLeftSided) factor = -1;
        while (isLeftSided ? drive.getPoseEstimate().getX() > juncPose.getX() : drive.getPoseEstimate().getX() < juncPose.getX()) {
            drive.setMotorPowers(-1, -1, -1, -1);
        }
        stopDrive();
        while (drive.getExternalHeading() < juncPose.getHeading()) {
            drive.setMotorPowers(-1*factor, 1*factor, -1*factor, 1*factor);
        }
        stopDrive();
    }

    public void strafeWithTime(boolean left, double time) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (driveTime.milliseconds() < time) {
            if (!left) {
                drive.setMotorPowers(1, -1, 1, -1);
            } else {
                drive.setMotorPowers(-1, 1, -1, 1);
            }
        }
        stopDrive();
    }

    public void strafeWithDistance(boolean left, double distance) {
        Pose2d initialPose = drive.getPoseEstimate();
        double initialX = initialPose.getX();
        double initialY = initialPose.getY();

        Trajectory traj;

        if (left) {
            traj = drive.trajectoryBuilder(initialPose)
                    .strafeLeft(distance)
                    .build();
        } else {
            traj = drive.trajectoryBuilder(initialPose)
                    .strafeRight(distance)
                    .build();
        }

        while (initialX > 5) {
            if (!left) {
                drive.setMotorPowers(1, -1, 1, -1);
            } else {
                drive.setMotorPowers(-1, 1, -1, 1);
            }
        }
        stopDrive();
    }

    public void stopDrive() {
        drive.setMotorPowers(0, 0,0 ,0);
    }
}

