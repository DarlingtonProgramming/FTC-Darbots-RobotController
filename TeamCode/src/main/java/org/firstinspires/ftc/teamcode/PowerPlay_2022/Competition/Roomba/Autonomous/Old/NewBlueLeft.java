package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Autonomous.Old;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.Color.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.RoombaDriveMethod;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "New Blue Left", group = "Scoring Blue")
public class NewBlueLeft extends LinearOpMode {

    private DcMotor Slide;
    private Servo Pinch;
    private WebcamName Webcam;
    private OpenCvCamera Camera;
    private ColorDetectionPipeline SleeveDetector;

    @Override
    public void runOpMode() {
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Pinch = hardwareMap.get(Servo.class, "Pinch");
        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);
        Pinch.setPosition(RoombaConstants.PINCH_MAX);

        initializeDetection();
        ColorDetectionPipeline.ParkingPosition visionResult = null;

        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = RoombaConstants.BLUE_LEFT_STARTING;
        drive.setPoseEstimate(startPose);

        RoombaDriveMethod driveMethod = new RoombaDriveMethod(drive, Slide, Pinch);

        final int SLIDE_INITIAL = Slide.getCurrentPosition();

        while (!opModeIsActive()) {
            visionResult = SleeveDetector.getPosition();
            telemetry.addData("Parking: ", visionResult);
            telemetry.update();
            sleep(75);
        }

        waitForStart();
        if (opModeIsActive()) {
            do {
                visionResult = SleeveDetector.getPosition();
            } while (visionResult == null);

            telemetry.addLine("Final Detection: " + visionResult);
            telemetry.update();

            TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.6);
                    })
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_INITIAL_STRAFE)
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_END_STRAFE)
                    .forward(2)
                    .build();
            drive.followTrajectorySequence(traj);

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 150, 1.0);
            driveMethod.setPinched(false);
            sleep(250);
            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 1.0);
            sleep(500);

            TrajectorySequence trajToStack = drive.trajectorySequenceBuilder(traj.end())
                    .addTemporalMarker(1, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[0], 0.8);
                    })
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_MIDPOINT)
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_CONE)
                    .build();
            drive.followTrajectorySequence(trajToStack);

            driveMethod.setPinched(true);
            sleep(700);
            driveMethod.slideTo(Slide.getCurrentPosition() + 550, 1);
            sleep(600);

            TrajectorySequence trajToJunction = drive.trajectorySequenceBuilder(trajToStack.end())
                    .addDisplacementMarker(() -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.7);
                    })
                    .lineToSplineHeading(new Pose2d(24, 10, toRadians(270)))
                    .forward(3)
                    .build();
            drive.followTrajectorySequence(trajToJunction);

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 150, 1.0);
            sleep(250);
            driveMethod.setPinched(false);
            sleep(500);

            TrajectorySequence trajToStack2 = drive.trajectorySequenceBuilder(trajToJunction.end())
                    .addTemporalMarker(0.2, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[1], 0.5);
                    })
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_MIDPOINT)
                    .lineToSplineHeading(RoombaConstants.BLUE_LEFT_CONE)
                    .build();
            drive.followTrajectorySequence(trajToStack2);

            driveMethod.setPinched(true);
            sleep(1000);
            driveMethod.slideTo(Slide.getCurrentPosition() + 500, 0.95);
            sleep(600);

            TrajectorySequence trajToJunction2 = drive.trajectorySequenceBuilder(trajToStack2.end())
                    .addDisplacementMarker(() -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.8);
                    })
                    .lineToSplineHeading(new Pose2d(24, 10, toRadians(270)))
                    .forward(3)
                    .build();
            drive.followTrajectorySequence(trajToJunction2);

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 150, 1.0);
            sleep(250);
            driveMethod.setPinched(false);
            sleep(500);
            driveMethod.slideTo(SLIDE_INITIAL, 0.95);
            sleep(1450);

            TrajectorySequence parkingTraj;
            switch (visionResult) {
                case LEFT: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(36, 10, toRadians(0)))
                            .lineToSplineHeading(new Pose2d(61, 10, toRadians(0)))
                            .build();
                    break;
                }
                case CENTER: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(38, 10, toRadians(0)))
                            .build();
                    break;
                }
                default: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(15, 10, toRadians(0)))
                            .build();
                    break;
                }
            }
            drive.followTrajectorySequence(parkingTraj);
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }

    private void initializeDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Camera = OpenCvCameraFactory.getInstance().createWebcam(Webcam, cameraMonitorViewId);
        SleeveDetector = new ColorDetectionPipeline();
        Camera.setPipeline(SleeveDetector);
        Camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Camera.startStreaming(RoombaConstants.OCV_STREAMING_WIDTH, RoombaConstants.OCV_STREAMING_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(Camera, 0);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }
}