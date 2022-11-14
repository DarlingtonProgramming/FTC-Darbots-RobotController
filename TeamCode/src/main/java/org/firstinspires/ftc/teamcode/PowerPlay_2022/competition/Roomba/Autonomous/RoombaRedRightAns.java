package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Settings.Roomba_Constants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Subsystems.RoombaDriveMethod;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Subsystems.RoombaDetectionBase;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name = "[3H] Roomba Red Right", group = "Scoring Red")
public class RoombaRedRightAns extends LinearOpMode {

    private DcMotor Slide;
    private Servo Pinch;
    private RoombaDetectionBase sleeveDetector = null;

    @Override
    public void runOpMode() {
        // Get devices from hardware map
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Pinch = hardwareMap.get(Servo.class, "Pinch");

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);

        // Initialize position
        Pinch.setPosition(Roomba_Constants.PINCH_MAX);

        // Detection
        try {
            sleeveDetector = new RoombaDetectionBase(Roomba_Constants.AllianceType.RED, hardwareMap, telemetry);
            sleeveDetector.activate();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Variables
        String visionResult = null;

        // Initialize roadrunner
        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = Roomba_Constants.RED_RIGHT_STARTING;
        drive.setPoseEstimate(startPose);

        // Initialize DriveMethod
        RoombaDriveMethod driveMethod = new RoombaDriveMethod(drive, Slide, Pinch);

        // Get initial slide height
        final int SLIDE_INITIAL = Slide.getCurrentPosition();

        // Initialize Trajectories
        Pose2d tempPose = startPose;
        ArrayList<TrajectorySequence> trajectories = new ArrayList<TrajectorySequence>();
        TrajectorySequence traj = drive.trajectorySequenceBuilder(tempPose)
                .lineToSplineHeading(Roomba_Constants.RED_RIGHT_INITIAL_STRAFE)
                .lineToSplineHeading(
                        Roomba_Constants.RED_RIGHT_END_STRAFE
                        //MecanumDrive_Roomba.getVelocityConstraint(32, DriveConstants_Roomba.MAX_ANG_VEL, DriveConstants_Roomba.TRACK_WIDTH),
                        //MecanumDrive_Roomba.getAccelerationConstraint(DriveConstants_Roomba.MAX_ACCEL)
                )
                .build();
        tempPose = traj.end();
        trajectories.add(traj);
        for (int i = 0; i < 2; i++) {
            int finalI = i;
            TrajectorySequence trajToStack = drive.trajectorySequenceBuilder(tempPose)
                    .addTemporalMarker(1.5, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + Roomba_Constants.CONE_HEIGHTS[finalI], 0.5);
                    })
                    .lineToSplineHeading(Roomba_Constants.RED_RIGHT_MIDPOINT)
                    .lineToSplineHeading(Roomba_Constants.RED_RIGHT_CONE.plus(new Pose2d(0, (i == 1 ? 0 : -2))))
                    .build();
            tempPose = trajToStack.end();
            trajectories.add(trajToStack);

            TrajectorySequence trajToJunction = drive.trajectorySequenceBuilder(tempPose)
                    .lineToSplineHeading(Roomba_Constants.RED_RIGHT_MIDPOINT)
                    .lineToSplineHeading(Roomba_Constants.RED_RIGHT_HIGH_JUNC)
                    .build();
            tempPose = trajToJunction.end();
            trajectories.add(trajToJunction);
        }

        while (!opModeIsActive()) {
            if (sleeveDetector.recogWithConfidence(Roomba_Constants.CONFIDENCE_LEVEL) != null) {
                visionResult = sleeveDetector.getCurrentResult();
                telemetry.addLine("Found: " + visionResult);
                telemetry.update();
            }
        }

        waitForStart();
        if (opModeIsActive()) {
            do {
                if (sleeveDetector.recogWithConfidence(Roomba_Constants.CONFIDENCE_LEVEL) != null) {
                    visionResult = sleeveDetector.getCurrentResult();
                    telemetry.addLine("Found: " + visionResult);
                    telemetry.update();
                }
            } while (visionResult == null && !isStopRequested());
            sleeveDetector.stopProcessing();

            telemetry.addLine("Found: " + visionResult);
            telemetry.update();

            driveMethod.slideTo(SLIDE_INITIAL + Roomba_Constants.SL_HIGH + 300, 0.45);
            drive.followTrajectorySequence(trajectories.get(0));

            driveMethod.setPinched(false);
            sleep(750);
            driveMethod.slideTo(SLIDE_INITIAL, 0.95);
            sleep(1550);

            drive.followTrajectorySequence(trajectories.get(1));

            driveMethod.setPinched(true);
            sleep(750);
            driveMethod.slideTo(Slide.getCurrentPosition() + 500, 0.95);
            sleep(300);

            drive.followTrajectorySequence(trajectories.get(2));

            driveMethod.slideTo(SLIDE_INITIAL + Roomba_Constants.SL_HIGH + 100, 0.95);
            sleep(1350);
            driveMethod.setPinched(false);
            sleep(500);
            driveMethod.slideTo(SLIDE_INITIAL, 0.9);
            sleep(1450);

            drive.followTrajectorySequence(trajectories.get(3));

            driveMethod.setPinched(true);
            sleep(750);
            driveMethod.slideTo(Slide.getCurrentPosition() + 500, 0.95);
            sleep(300);

            drive.followTrajectorySequence(trajectories.get(4));

            driveMethod.slideTo(SLIDE_INITIAL + Roomba_Constants.SL_HIGH + 100, 0.95);
            sleep(1350);
            driveMethod.setPinched(false);
            sleep(500);
            driveMethod.slideTo(SLIDE_INITIAL, 0.95);
            sleep(1450);

            if (visionResult.equals(Roomba_Constants.LABELS[0])) { // Eyes
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(-33, 11, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            } else if (visionResult.equals(Roomba_Constants.LABELS[1])) { // Bat
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(-36, 11, toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-57, 11, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            } else if (visionResult.equals(Roomba_Constants.LABELS[2])) { // Lantern
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(Roomba_Constants.RED_RIGHT_MIDPOINT)
                        .lineToSplineHeading(new Pose2d(-14, 11, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}