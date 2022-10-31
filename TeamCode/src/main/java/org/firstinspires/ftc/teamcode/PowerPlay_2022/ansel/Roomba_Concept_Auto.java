package org.firstinspires.ftc.teamcode.PowerPlay_2022.ansel;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Roomba_Constants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.detection.PPDetector;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.detection.classification.Classifier;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.FieldConstant;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Roomba Concept Auto", group = "Competition")
public class Roomba_Concept_Auto extends LinearOpMode {

    private DcMotor LF, RF, LB, RB, Slide;
    private CRServo Turn;
    private Servo Pinch;

    @Override
    public void runOpMode() {
        // Get devices from hardware map
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "Slide");

        Turn = hardwareMap.get(CRServo.class, "Turn");
        Pinch = hardwareMap.get(Servo.class, "Pinch");

        // Initialize devices
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize position
        Pinch.setPosition(Roomba_Constants.PINCH_MAX);


        // Initialize roadrunner
        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = new Pose2d(31, 64, toRadians(270));
        drive.setPoseEstimate(startPose);

        waitForStart();
        final int SLIDE_INITIAL = Slide.getCurrentPosition();
        if (opModeIsActive()) {
            TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(22)
                    .lineToSplineHeading(new Pose2d(16, 8, toRadians(315)))
                    .build();
            slideTo(SLIDE_INITIAL + Roomba_Constants.SL_HIGH + 280, 0.45);
            drive.followTrajectorySequence(traj);
            while (drive.isBusy()) sleep(250);

            setPinched(false);
            sleep(750);

            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                    .back(9)
                    .addDisplacementMarker(1, () -> {
                        slideTo(SLIDE_INITIAL + 420, 0.5);
                    })
                    .lineToSplineHeading(new Pose2d(64, 12, toRadians(0)))
                    .build();

            drive.followTrajectorySequence(traj2);
            while (drive.isBusy()) sleep(250);

            setPinched(true);
            slideTo(SLIDE_INITIAL + Roomba_Constants.SL_LOW + 150, 0.9);
            sleep(750);

            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .lineToSplineHeading(new Pose2d(48.5, 15, toRadians(90)))
                    .build();
            drive.followTrajectorySequence(traj3);
            while (drive.isBusy()) sleep(250);

            setPinched(false);
            sleep(750);

            TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                    .addDisplacementMarker(() -> {
                        slideTo(SLIDE_INITIAL + 315, 0.9);
                    })
                    .lineToSplineHeading(new Pose2d(64, 12, toRadians(0)))
                    .build();
            drive.followTrajectorySequence(traj4);
            while (drive.isBusy()) sleep(250);

            setPinched(true);
            slideTo(SLIDE_INITIAL + Roomba_Constants.SL_LOW + 150, 0.9);
            sleep(750);

            TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                    .lineToSplineHeading(new Pose2d(48.5, 15, toRadians(90)))
                    .build();
            drive.followTrajectorySequence(traj5);
            while (drive.isBusy()) sleep(250);

            setPinched(false);
            slideTo(SLIDE_INITIAL, 0.5);

            /*
            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                    .forward(3)
                    .addTemporalMarker(1.5, () -> {
                        slideTo(Slide.getCurrentPosition() - 150, 0.9);
                    })
                    .addTemporalMarker(0.5, () -> {
                        setPinched(false);
                    })
                    .addTemporalMarker(1, () -> {
                        slideTo(Slide.getCurrentPosition() + 150, 0.9);
                    })
                    .build();
            drive.followTrajectorySequence(traj2);
            while (drive.isBusy()) sleep(250);

            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                    .back(9)
                    .addTemporalMarker(0.5, () -> {
                        slideTo(SLIDE_INITIAL, 0.5);
                    })
                    .build();
            drive.followTrajectorySequence(traj3);
            while (drive.isBusy()) sleep(250);

            sleep(2500);

            TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                    .lineToSplineHeading(new Pose2d(58.5, 12, toRadians(0)))
                    .build();
            drive.followTrajectorySequence(traj3);
            */
        }
    }

    private void slideTo(int targetPosition, double power) {
        Slide.setTargetPosition(targetPosition);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);
    }

    private void setPinched(boolean pinched) {
        if (pinched)
            Pinch.setPosition(Roomba_Constants.PINCH_MAX);
        else
            Pinch.setPosition(Roomba_Constants.PINCH_MIN);
        sleep(500);
    }
}