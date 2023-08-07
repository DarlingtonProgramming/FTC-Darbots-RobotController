package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Striker.Autonomous;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.Color.ColorDetectionPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Striker;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "TestAuto BLUE LEFT", group = "Test")
public class TestAuto extends LinearOpMode {

    private DcMotor Slide, Turn;
    private Servo Pinch;
    private WebcamName Webcam;
    private OpenCvCamera Camera;
    private ColorDetectionPipeline SleeveDetector;

    @Override
    public void runOpMode() {
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Turn = hardwareMap.get(DcMotor.class, "Turn");
        Pinch = hardwareMap.get(Servo.class, "Pinch");
        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        Turn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MecanumDrive_Striker drive = new MecanumDrive_Striker(hardwareMap);
        Pose2d start = new Pose2d(30.5,62, toRadians(180));
        drive.setPoseEstimate(start);

        initializeDetection();
        ColorDetectionPipeline.ParkingPosition visionResult = null;

        final double PINCH_CLOSED = 0.68;
        final double PINCH_OPEN = 1;
        final int SLIDE_INITIAL = 0;
        final int TURN_INITIAL = Turn.getCurrentPosition();

        Pinch.setPosition(PINCH_CLOSED);
        sleep(1000);
        Slide.setTargetPosition(SLIDE_INITIAL + 130);  //slide sets 3000 moves to high junction
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(1);

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

            TrajectorySequence juncTraj2 = drive.trajectorySequenceBuilder(start)
                    .forward(22)       // goes forward
                    .addDisplacementMarker( () -> {
                        Slide.setTargetPosition(SLIDE_INITIAL + 5700);  //slide sets 3000 moves to high junction
                        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Slide.setPower(1);
                    })
                    .lineToSplineHeading(new Pose2d(13,17, toRadians(180)))
                    .build();
            drive.followTrajectorySequence(juncTraj2);
            while (drive.isBusy()) sleep(30);

            /*
            Turn.setTargetPosition(TURN_INITIAL + 0);
            Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turn.setPower(1);
            sleep(30);


             */

            TrajectorySequence juncTrajj2 = drive.trajectorySequenceBuilder(juncTraj2.end())
                    .forward(1)
                    .build();
            drive.followTrajectorySequence(juncTrajj2);
            while (drive.isBusy()) sleep(190);


            sleep(400);
            Slide.setTargetPosition(SLIDE_INITIAL + 500);  //slide sets 3000 moves to high junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(1);
            sleep(400);
            Pinch.setPosition(PINCH_OPEN);
            sleep(400);

            TrajectorySequence juncTraj3 = drive.trajectorySequenceBuilder(juncTrajj2.end())
                    .lineToSplineHeading(new Pose2d(12,5, toRadians(180)))
                    .build();
            drive.followTrajectorySequence(juncTraj3);
            while (drive.isBusy()) sleep(90);

                                                //305
            Turn.setTargetPosition(TURN_INITIAL + 305);
            Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turn.setPower(1);
            sleep(1300);

            TrajectorySequence juncTraj33 = drive.trajectorySequenceBuilder(juncTraj3.end())
                    .lineToSplineHeading(new Pose2d(63, 5, toRadians(180)))
                    .build();
            drive.followTrajectorySequence(juncTraj33);
            while (drive.isBusy()) sleep(90);

            sleep(200);
            Pinch.setPosition(PINCH_CLOSED);      //pinch closes
            sleep(1000);
            Slide.setTargetPosition(SLIDE_INITIAL + 1750); //slide shifts up
            Slide.setPower(0.9);
            sleep(300);

            TrajectorySequence juncTraj4 = drive.trajectorySequenceBuilder(juncTraj33.end())
                    .forward(13)
                    .strafeRight(4)
                    .build();
            drive.followTrajectorySequence(juncTraj4);
            while (drive.isBusy()) sleep(190);

            Turn.setTargetPosition(TURN_INITIAL + 457);
            Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turn.setPower(1);
            sleep(1300);

            Slide.setTargetPosition(SLIDE_INITIAL + 450); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(300);
            Pinch.setPosition(PINCH_OPEN);

            sleep(100);
            Turn.setTargetPosition(TURN_INITIAL + 305);
            Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turn.setPower(1);
            sleep(1300);

            TrajectorySequence juncTraj5 = drive.trajectorySequenceBuilder(juncTraj4.end())
                    .strafeLeft(4)
                    .lineToSplineHeading(new Pose2d(63, 5, toRadians(180)))
                    .build();
            drive.followTrajectorySequence(juncTraj5);
            while (drive.isBusy()) sleep(90);

            Pinch.setPosition(PINCH_CLOSED);      //pinch closes
            sleep(1000);
            Slide.setTargetPosition(SLIDE_INITIAL + 1750); //slide shifts up
            Slide.setPower(0.9);

            TrajectorySequence juncTraj6 = drive.trajectorySequenceBuilder(juncTraj5.end())
                    .forward(13)
                    .strafeRight(4)
                    .build();
            drive.followTrajectorySequence(juncTraj6);
            while (drive.isBusy()) sleep(90);

            Turn.setTargetPosition(TURN_INITIAL + 457);
            Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turn.setPower(1);
            sleep(1300);

            Slide.setTargetPosition(SLIDE_INITIAL + 400); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(300);
            Pinch.setPosition(PINCH_OPEN);

            sleep(100);
            Turn.setTargetPosition(TURN_INITIAL + 305);
            Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turn.setPower(1);
            sleep(1300);

            TrajectorySequence juncTraj7 = drive.trajectorySequenceBuilder(juncTraj6.end())
                    .strafeLeft(4)
                    .lineToSplineHeading(new Pose2d(63, 5, toRadians(180)))
                    .build();
            drive.followTrajectorySequence(juncTraj7);
            while (drive.isBusy()) sleep(90);

            Pinch.setPosition(PINCH_CLOSED);      //pinch closes
            sleep(1000);
            Slide.setTargetPosition(SLIDE_INITIAL + 1750); //slide shifts up
            Slide.setPower(0.9);

            TrajectorySequence juncTraj8 = drive.trajectorySequenceBuilder(juncTraj7.end())
                    .forward(13)
                    .strafeRight(4)
                    .build();
            drive.followTrajectorySequence(juncTraj8);
            while (drive.isBusy()) sleep(90);

            Turn.setTargetPosition(TURN_INITIAL + 457);
            Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turn.setPower(1);
            sleep(1300);

            Slide.setTargetPosition(SLIDE_INITIAL + 350); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(300);
            Pinch.setPosition(PINCH_OPEN);







/*
            sleep(300);
            Turn.setTargetPosition(TURN_INITIAL + 0);
            Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turn.setPower(1);
            sleep(500);
            Slide.setTargetPosition(SLIDE_INITIAL + 0); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(1000);
*/

            TrajectorySequence parkingTraj;
            switch (visionResult) {
                case LEFT: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .back(15)
                            .build();
                    break;
                }
                case CENTER: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(13)
                            .build();
                    break;
                }
                default: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(37)
                            .build();
                    break;
                }
            }
            drive.followTrajectorySequence(parkingTraj);
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


}
