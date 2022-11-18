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

@Disabled
@Autonomous(name = "[3H] Roomba Blue Left", group = "Scoring Blue")
public class RoombaBlueLeftAns extends LinearOpMode {

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
        Pose2d startPose = RoombaConstants.BLUE_LEFT_STARTING;
        drive.setPoseEstimate(startPose);

        // Initialize DriveMethod
        RoombaDriveMethod driveMethod = new RoombaDriveMethod(drive, Slide, Pinch);

        // Get initial slide height
        final int SLIDE_INITIAL = Slide.getCurrentPosition();

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

            if (sleeveDetector != null) {
                sleeveDetector.stopProcessing();
            }

            telemetry.addLine("Found: " + visionResult);
            telemetry.update();

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.45);

            TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_INITIAL_STRAFE)
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_END_STRAFE)
                    .forward(3)
                    .build();
            drive.followTrajectorySequence(traj);
            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 100, 0.9);
            sleep(500);
            driveMethod.setPinched(false);
            sleep(500);

            TrajectorySequence trajToStack = drive.trajectorySequenceBuilder(traj.end())
                    .back(3)
                    .addTemporalMarker(1.8, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[0], 0.7);
                    })
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_MIDPOINT)
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_CONE)
                    .build();
            drive.followTrajectorySequence(trajToStack);

            driveMethod.setPinched(true);
            sleep(1000);
            driveMethod.slideTo(Slide.getCurrentPosition() + 500, 0.95);
            sleep(600);

            TrajectorySequence trajToJunction = drive.trajectorySequenceBuilder(trajToStack.end())
                    .lineToSplineHeading(new Pose2d(29, 10, toRadians(270)))
                    .build();
            drive.followTrajectorySequence(trajToJunction);

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.95);
            sleep(1350);
            driveMethod.setPinched(false);
            sleep(500);
            driveMethod.slideTo(SLIDE_INITIAL, 0.9);
            sleep(1450);

            TrajectorySequence trajToStack2 = drive.trajectorySequenceBuilder(trajToJunction.end())
                    .addTemporalMarker(1.5, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[1], 0.5);
                    })
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_MIDPOINT)
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_CONE.plus((new Pose2d(1, -1))))
                    .build();
            drive.followTrajectorySequence(trajToStack2);

            driveMethod.setPinched(true);
            sleep(1000);
            driveMethod.slideTo(Slide.getCurrentPosition() + 500, 0.95);
            sleep(600);

            TrajectorySequence trajToJunction2 = drive.trajectorySequenceBuilder(trajToStack2.end())
                    .lineToSplineHeading(new Pose2d(28, 10, toRadians(270)))
                    .build();
            drive.followTrajectorySequence(trajToJunction2);

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.95);
            sleep(1350);
            driveMethod.setPinched(false);
            sleep(500);
            driveMethod.slideTo(SLIDE_INITIAL, 0.95);
            sleep(1450);

            if (visionResult.equals(RoombaConstants.LABELS[0])) { // Eyes1
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(38, 10, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            } else if (visionResult.equals(RoombaConstants.LABELS[1])) { // Bat
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(15, 10, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            } else if (visionResult.equals(RoombaConstants.LABELS[2])) { // Lantern
                TrajectorySequence traj6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(36, 10, toRadians(0)))
                        .lineToSplineHeading(new Pose2d(61, 10, toRadians(0)))
                        .build();
                drive.followTrajectorySequence(traj6);
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
}