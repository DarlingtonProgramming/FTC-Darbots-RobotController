package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Autonomous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.Color.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.RecordingPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.RoombaDriveMethod;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Roomba BL Auto", group = "0")
public class RoombaNewAuto extends LinearOpMode {

    enum State {
        STRAFE_TO_JUNC,
        TO_STACK,
        TO_JUNCTION,
        PARK,
        IDLE
    }
    private State currentState = null;

    private DcMotor Slide;
    private Servo Pinch;

    private ColorDetectionPipeline SleeveDetector;
    private OpenCvWebcam Camera;
    private WebcamName Webcam;

    @Override
    public void runOpMode() {
        // Get devices from hardware map
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Pinch = hardwareMap.get(Servo.class, "Pinch");

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);
        Pinch.setPosition(RoombaConstants.PINCH_MAX);

        // Get initial slide height
        final int SLIDE_INITIAL = Slide.getCurrentPosition();

        // Initialize roadrunner
        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = RoombaConstants.BLUE_LEFT_STARTING;
        drive.setPoseEstimate(startPose);

        // Initialize DriveMethod
        RoombaDriveMethod driveMethod = new RoombaDriveMethod(drive, Slide, Pinch);

        // Initialize trajectories
        TrajectorySequence t_strafe_to_junc = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(34, 55.5, toRadians(180)), toRadians(0))
                .lineToSplineHeading(new Pose2d(33, 0, toRadians(180)))
                .addSpatialMarker(new Vector2d(33, 0), () -> {
                    driveMethod.setPinched(false);
                })
                .build();
        TrajectorySequence t_to_stack_one = drive.trajectorySequenceBuilder(t_strafe_to_junc.end())
                .addDisplacementMarker(() -> {
                    driveMethod.slideTo(SLIDE_INITIAL + 420, 0.8);
                })
                .lineToSplineHeading(new Pose2d(33, 12, toRadians(0)))
                .lineToSplineHeading(new Pose2d(63, 12, toRadians(0)))
                .addSpatialMarker(new Vector2d(63, 12), () -> {
                    driveMethod.setPinched(true);
                    driveMethod.slideTo(Slide.getCurrentPosition() + 500, 0.9);
                })
                .build();
        TrajectorySequence t_to_junc_one = drive.trajectorySequenceBuilder(t_strafe_to_junc.end())
                .addDisplacementMarker(() -> {
                    driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.8);
                })
                .lineToSplineHeading(new Pose2d(23.5, 13, toRadians(270)))
                .forward(4)
                .addSpatialMarker(new Vector2d(23.5, 9.5), () -> {
                    driveMethod.slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 150, 0.95);
                })
                .addSpatialMarker(new Vector2d(23.5, 9.3), () -> {
                    driveMethod.setPinched(false);
                })
                .back(4)
                .build();

        // Initialize sleeve detection
        initOCVDetection();
        ColorDetectionPipeline.ParkingPosition visionResult = null;

        // Loop through sleeve detector while initialized
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Sleeve Detector", SleeveDetector.getPosition());
            telemetry.update();
            sleep(50);
        }

        visionResult = SleeveDetector.getPosition();
        currentState = State.STRAFE_TO_JUNC;

        int[] stateIterations = { 0, 0 };

        if (opModeIsActive()) {
            drive.followTrajectorySequence(t_strafe_to_junc);
        }

        while (opModeIsActive() && !isStopRequested()) {
            /*
            switch (currentState) {
                case STRAFE_TO_JUNC:
                        currentState = State.TO_STACK;
                        drive.followTrajectorySequenceAsync(t_strafe_to_junc);
                    break;
                case TO_STACK:
                    stateIterations[0]++;
                    if (!drive.isBusy()) {
                        currentState = State.TO_JUNCTION;
                        drive.followTrajectorySequenceAsync(t_to_stack_one);
                    }
                    break;
                case TO_JUNCTION:
                    stateIterations[1]++;
                    if (!drive.isBusy()) {
                        currentState = State.PARK;
                        drive.followTrajectorySequenceAsync(t_to_junc_one);
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        TrajectorySequence park;
                        switch (visionResult) {
                            case LEFT:
                                park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .lineToSplineHeading(new Pose2d(36, 10, toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(61, 10, toRadians(0)))
                                        .build();
                                break;
                            case CENTER:
                                park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .lineToSplineHeading(new Pose2d(38, 10, toRadians(0)))
                                        .build();
                                break;
                            default:
                                park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .lineToSplineHeading(new Pose2d(15, 10, toRadians(0)))
                                        .build();
                                break;
                        }
                        driveMethod.slideTo(SLIDE_INITIAL, 0.8);
                        drive.followTrajectorySequenceAsync(park);
                    }
                    break;
                case IDLE:
                    break;
            }

             */
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("Got", visionResult);
            telemetry.addData("X Position", poseEstimate.getX());
            telemetry.addData("Y Position", poseEstimate.getY());
            telemetry.addData("Heading", poseEstimate.getHeading());
            telemetry.addData("Slide Height", Slide.getCurrentPosition());
            telemetry.update();
        }
    }
    private void initOCVDetection() {
        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

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
    private void initOCVRecording() {
        this.Camera.setPipeline(new RecordingPipeline(this.Camera));
        this.Camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        this.Camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        this.Camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Camera.startStreaming(RoombaConstants.OCV_RECORDING_WIDTH, RoombaConstants.OCV_RECORDING_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Could not start recording.");
                telemetry.update();
            }
        });
    }
}