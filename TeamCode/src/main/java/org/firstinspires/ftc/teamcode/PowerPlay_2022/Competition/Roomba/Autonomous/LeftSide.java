package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Autonomous;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.Color.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.DriveConstants_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "bocchi the left \uD83E\uDD76\uD83E\uDD76\uD83E\uDD76", group = "1")
public class LeftSide extends LinearOpMode {
    private final boolean TRY_COLOR = false;

    private DcMotor Slide;
    private Servo Pinch;
    private WebcamName Webcam;
    private OpenCvCamera Camera;
    private DistanceSensor Distance;
    private ColorSensor Color;
    private ColorDetectionPipeline SleeveDetector;

    @Override
    public void runOpMode() {
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Pinch = hardwareMap.get(Servo.class, "Pinch");
        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        Color = hardwareMap.get(ColorSensor.class,"Color");

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);
        Pinch.setPosition(RoombaConstants.PINCH_MAX);
        Color.enableLed(false);

        initializeDetection();
        ColorDetectionPipeline.ParkingPosition visionResult = null;

        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = new Pose2d(30.5, 62, toRadians(270));
        drive.setPoseEstimate(startPose);

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

            TrajectorySequence mediumJunction = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> {
                        slideTo(SLIDE_INITIAL + RoombaConstants.SL_MEDIUM);
                    })
                    .lineTo(new Vector2d(startPose.getX() + 1.5, startPose.getY() - 3))
                    .lineToSplineHeading(new Pose2d(39, 24, toRadians(180)))
                    .forward(6)
                    .build();
            drive.followTrajectorySequence(mediumJunction);
            while (drive.isBusy()) sleep(90);

            slideTo(SLIDE_INITIAL + RoombaConstants.SL_MEDIUM - 300);
            sleep(50);
            Pinch.setPosition(RoombaConstants.PINCH_MIN);
            sleep(60);

            TrajectorySequence toStackFirstCone = drive.trajectorySequenceBuilder(mediumJunction.end())
                    .addDisplacementMarker(3, () -> {
                        Pinch.setPosition(RoombaConstants.PINCH_MIN);
                    })
                    .addDisplacementMarker(5, () -> {
                        Slide.setTargetPosition(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[0]);
                        Slide.setPower(0.9);
                    })
                    .back(5)
                    .strafeLeft(13.5)
                    .turn(toRadians(-180))
                    .forward(14)
                    .build();
            drive.followTrajectorySequence(toStackFirstCone);
            while (drive.isBusy()) sleep(100);

            lineWithSensors(drive);
            Pinch.setPosition(RoombaConstants.PINCH_MAX);
            sleep(700);
            slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.6);
            sleep(300);

            TrajectorySequence toHighJunctionFirstCone = drive.trajectorySequenceBuilder(toStackFirstCone.end())
                    .back(3.5)
                    .lineToSplineHeading(
                            new Pose2d(24,10, toRadians(270)),
                            MecanumDrive_Roomba.getVelocityConstraint(45, DriveConstants_Roomba.MAX_ANG_VEL, DriveConstants_Roomba.TRACK_WIDTH),
                            MecanumDrive_Roomba.getAccelerationConstraint(45)
                    )
                    .forward(3.5)
                    .build();
            drive.followTrajectorySequence(toHighJunctionFirstCone);
            while (drive.isBusy()) sleep(100);

            slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 350, 0.2);
            sleep(200);
            Pinch.setPosition(RoombaConstants.PINCH_MIN);
            sleep(200);

            TrajectorySequence toStackSecondCone = drive.trajectorySequenceBuilder(toHighJunctionFirstCone.end())
                    .addDisplacementMarker(2, () -> {
                        slideTo(SLIDE_INITIAL + RoombaConstants.CONE_HEIGHTS[1]);
                    })
                    .back(3.5)
                    .turn(toRadians(90))
                    .forward(25)
                    .build();
            drive.followTrajectorySequence(toStackSecondCone);
            while (drive.isBusy()) sleep(100);

            lineWithSensors(drive);
            Pinch.setPosition(RoombaConstants.PINCH_MAX);
            sleep(700);
            slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH, 0.6);
            sleep(300);

            TrajectorySequence toHighJunctionSecondCone = drive.trajectorySequenceBuilder(toStackSecondCone.end())
                    .back(3.5)
                    .lineToSplineHeading(
                            new Pose2d(22,10,toRadians(270)),
                            MecanumDrive_Roomba.getVelocityConstraint(45, DriveConstants_Roomba.MAX_ANG_VEL, DriveConstants_Roomba.TRACK_WIDTH),
                            MecanumDrive_Roomba.getAccelerationConstraint(45)
                    )
                    .forward(3)
                    .build();
            drive.followTrajectorySequence(toHighJunctionSecondCone);
            while (drive.isBusy()) sleep(100);

            slideTo(SLIDE_INITIAL + RoombaConstants.SL_HIGH - 350, 0.2);
            sleep(200);
            Pinch.setPosition(RoombaConstants.PINCH_MIN);
            sleep(200);

            TrajectorySequence lineWithMiddle = drive.trajectorySequenceBuilder(toHighJunctionSecondCone.end())
                    .back(3)
                    .build();
            drive.followTrajectorySequence(lineWithMiddle);
            while (drive.isBusy()) sleep(100);

            slideTo(SLIDE_INITIAL);

            TrajectorySequence parkingTraj;
            switch (visionResult) {
                case LEFT: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .strafeLeft(34)
                            .build();
                    drive.followTrajectorySequence(parkingTraj);
                    break;
                }
                case CENTER: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .strafeLeft(15)
                            .build();
                    drive.followTrajectorySequence(parkingTraj);
                    break;
                }
                default: {
                     parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .strafeRight(10)
                            .build();
                    drive.followTrajectorySequence(parkingTraj);
                    break;
                }
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
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

    private void slideTo(int target) {
        Slide.setTargetPosition(target);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.9);
    }

    private void slideTo(int target, double power) {
        Slide.setTargetPosition(target);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);
    }

    private void lineWithSensors(MecanumDrive_Roomba drive) {
        if (TRY_COLOR) {
            Color.enableLed(true);
            do {
                drive.setMotorPowers(0.2, -0.2, 0.2, -0.2);
            } while (Color.blue() < 130);
            drive.setMotorPowers(0, 0, 0, 0);
            Color.enableLed(false);
        }

        do {
            if (Distance.getDistance(DistanceUnit.MM) > 90) {
                drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
            } else if (Distance.getDistance(DistanceUnit.MM) > 75) {
                drive.setMotorPowers(0.18, 0.18, 0.18, 0.18);
            } else {
                drive.setMotorPowers(0.1, 0.1, 0.1, 0.1);
            }
        } while (Distance.getDistance(DistanceUnit.MM) > 43);

        drive.setMotorPowers(0, 0, 0, 0);
    }
}