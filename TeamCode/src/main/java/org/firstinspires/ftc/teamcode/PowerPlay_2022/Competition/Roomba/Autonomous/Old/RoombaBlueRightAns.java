package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Autonomous.Old;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.RoombaDriveMethod;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.RoombaTFDetectionBase;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "[3H] Roomba Blue Right", group = "Scoring Blue")
public class RoombaBlueRightAns extends LinearOpMode {

    private DcMotor Slide;
    private Servo Pinch;
    private RoombaTFDetectionBase sleeveDetector = null;

    @Override
    public void runOpMode() {
        // Get devices from hardware map
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Pinch = hardwareMap.get(Servo.class, "Pinch");

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);

        // Initialize position
        Pinch.setPosition(RoombaConstants.PINCH_MAX);

        // Detection
        try {
            sleeveDetector = new RoombaTFDetectionBase(RoombaConstants.AllianceType.BLUE, hardwareMap, telemetry);
            sleeveDetector.activate();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Variables
        String visionResult = null;

        // Initialize roadrunner
        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = RoombaConstants.RED_RIGHT_STARTING;
        drive.setPoseEstimate(startPose);

        // Initialize DriveMethod
        RoombaDriveMethod driveMethod = new RoombaDriveMethod(drive, Slide, Pinch);

        // Get initial slide height
        final int SLIDE_INITIAL = Slide.getCurrentPosition();

        // Initialize Trajectories
        Pose2d tempPose = startPose;
        ArrayList<TrajectorySequence> trajectories = new ArrayList<TrajectorySequence>();
        TrajectorySequence traj = drive.trajectorySequenceBuilder(tempPose)
                .lineToSplineHeading(RoombaConstants.RED_RIGHT_INITIAL_STRAFE)
                .lineToSplineHeading(
                        RoombaConstants.RED_RIGHT_END_STRAFE
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
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[finalI], 0.5);
                    })
                    .lineToSplineHeading(RoombaConstants.RED_RIGHT_MIDPOINT)
                    .lineToSplineHeading(RoombaConstants.RED_RIGHT_CONE)
                    .build();
            tempPose = trajToStack.end();
            trajectories.add(trajToStack);

            TrajectorySequence trajToJunction = drive.trajectorySequenceBuilder(tempPose)
                    .lineToSplineHeading(RoombaConstants.RED_RIGHT_MIDPOINT)
                    .lineToSplineHeading(RoombaConstants.RED_RIGHT_HIGH_JUNC.plus(new Pose2d(0, (i == 1 ? -2 : 0))))
                    .build();
            tempPose = trajToJunction.end();
            trajectories.add(trajToJunction);
        }

        while (!opModeIsActive()) {
            if (sleeveDetector.recogWithConfidence(RoombaConstants.CONFIDENCE_LEVEL) != null) {
                visionResult = sleeveDetector.getCurrentResult();
                telemetry.addLine("Found: " + visionResult);
                telemetry.update();
            }
        }

        waitForStart();
        if (opModeIsActive()) {
            do {
                if (sleeveDetector.recogWithConfidence(RoombaConstants.CONFIDENCE_LEVEL) != null) {
                    visionResult = sleeveDetector.getCurrentResult();
                    telemetry.addLine("Found: " + visionResult);
                    telemetry.update();
                }
            } while (visionResult == null && !isStopRequested());
            sleeveDetector.stopProcessing();

            telemetry.addLine("Found: " + visionResult);
            telemetry.update();

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH + 500, 0.45);
            drive.followTrajectorySequence(trajectories.get(0));

            //drive.setPoseEstimate(Roomba_Constants.RED_RIGHT_END_STRAFE.plus(new Pose2d(-2))); //new Pose2d(34, 0, toRadians(180))
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

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.95);
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

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.95);
            sleep(1350);
            driveMethod.setPinched(false);
            sleep(500);
            driveMethod.slideTo(SLIDE_INITIAL, 0.95);
            sleep(1450);

            if (visionResult.equals(RoombaConstants.LABELS[0])) { // Eyes
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(-33, 11, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            } else if (visionResult.equals(RoombaConstants.LABELS[1])) { // Bat
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(-14, 11, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            } else if (visionResult.equals(RoombaConstants.LABELS[2])) { // Lantern
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(-36, 11, toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-57, 11, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}