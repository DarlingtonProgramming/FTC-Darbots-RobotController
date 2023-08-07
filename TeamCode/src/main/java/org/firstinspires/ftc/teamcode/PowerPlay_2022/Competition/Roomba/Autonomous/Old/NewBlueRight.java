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
@Autonomous(name = "New Blue Right", group = "Scoring Blue")
public class NewBlueRight extends LinearOpMode {

    private DcMotor Slide;
    private Servo Pinch;
    private WebcamName Webcam;
    private OpenCvCamera Camera;
    private ColorDetectionPipeline SleeveDetector;

    @Override
    public void runOpMode() {
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Pinch = hardwareMap.get(Servo.class, "Pinch");

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);
        Pinch.setPosition(RoombaConstants.PINCH_MAX);

        initializeDetection();
        ColorDetectionPipeline.ParkingPosition visionResult = null;

        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = new Pose2d(-40, 64.25, toRadians(270));
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
                    .addDisplacementMarker(4, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.7);
                    })
                    .lineToSplineHeading(new Pose2d(-11, 60, toRadians(270)))
                    .lineToSplineHeading(new Pose2d(-6, 32, toRadians(318)))
                    .build();
            drive.followTrajectorySequence(traj);

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 250, 0.65);
            sleep(650);
            Pinch.setPosition(RoombaConstants.PINCH_MIN);
            sleep(500);
            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 1.0);
            sleep(600);

            TrajectorySequence trajToStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[0], 0.5);
                    })
                    .lineToSplineHeading(new Pose2d(-11, 43, toRadians(318)))
                    .lineToSplineHeading(new Pose2d(-11, 14, toRadians(180)))
                    .lineToSplineHeading(new Pose2d(-58.2, 15.5, toRadians(180)))
                    .build();
            drive.followTrajectorySequence(trajToStack);

            Pinch.setPosition(RoombaConstants.PINCH_MAX);
            sleep(1000);
            driveMethod.slideTo(Slide.getCurrentPosition() + 550, 1);
            sleep(600);

            TrajectorySequence trajToJunction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(3, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.7);
                    })
                    .lineToSplineHeading(new Pose2d(-19.7, 14.5, toRadians(270)))
                    .forward(3)
                    .build();
            drive.followTrajectorySequence(trajToJunction);

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 200, 0.6);
            sleep(650);
            Pinch.setPosition(RoombaConstants.PINCH_MIN);
            sleep(600);

            TrajectorySequence trajToStack2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(4.5)
                    .addTemporalMarker(0.2, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[1], 0.5);
                    })
                    .lineToSplineHeading(new Pose2d(-58.2, 15, toRadians(180)).plus(new Pose2d(0, -2)))
                    .build();
            drive.followTrajectorySequence(trajToStack2);

            Pinch.setPosition(RoombaConstants.PINCH_MAX);
            sleep(1000);
            driveMethod.slideTo(Slide.getCurrentPosition() + 500, 0.95);
            sleep(600);

            TrajectorySequence trajToJunction2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(3, () -> {
                        driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.8);
                    })
                    .lineToSplineHeading(new Pose2d(-19.6, 14.5, toRadians(270)))
                    .forward(4)
                    .build();
            drive.followTrajectorySequence(trajToJunction2);

            driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 200, 0.6);
            sleep(650);
            Pinch.setPosition(RoombaConstants.PINCH_MIN);
            sleep(600);

            TrajectorySequence parkingTraj;
            switch (visionResult) {
                case RIGHT: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(0.5, () -> {
                                driveMethod.slideTo(SLIDE_INITIAL, 0.6);
                            })
                            .back(4)
                            .lineToSplineHeading(new Pose2d(-36, 14, toRadians(0)))
                            .lineToSplineHeading(new Pose2d(-56, 14, toRadians(0)))
                            .build();
                    break;
                }
                case CENTER: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(0.5, () -> {
                                driveMethod.slideTo(SLIDE_INITIAL, 0.6);
                            })
                            .back(4)
                            .lineToSplineHeading(new Pose2d(-32, 15, toRadians(270)))
                            .build();
                    break;
                }
                default: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(0.5, () -> {
                                driveMethod.slideTo(SLIDE_INITIAL, 0.6);
                            })
                            .back(4)
                            .lineToSplineHeading(new Pose2d(-8, 16, toRadians(270)))
                            .turn(toRadians(180))
                            .build();
                    break;
                }
            }
            drive.followTrajectorySequence(parkingTraj);
            PoseStorage.currentPose = drive.getPoseEstimate();
            sleep(200);
        }
    }

    private void initializeDetection() {
        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Camera = OpenCvCameraFactory.getInstance().createWebcam(Webcam, cameraMonitorViewId);
        SleeveDetector = new ColorDetectionPipeline();
        Camera.setPipeline(SleeveDetector);
        Camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Camera.startStreaming(RoombaConstants.OCV_STREAMING_WIDTH, RoombaConstants.OCV_STREAMING_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                // FtcDashboard.getInstance().startCameraStream(Camera, 0);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }
}