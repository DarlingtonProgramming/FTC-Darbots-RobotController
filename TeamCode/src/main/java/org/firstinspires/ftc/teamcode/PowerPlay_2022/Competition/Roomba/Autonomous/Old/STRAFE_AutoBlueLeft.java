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
@Autonomous(name = "STRAFE_AutoBlueLeft", group = "Scoring Blue")
public class STRAFE_AutoBlueLeft extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(30.5, 62, toRadians(270));
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

            Slide.setTargetPosition(SLIDE_INITIAL + 2800);  //slide sets 2750 moves to high junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.3);
            sleep(50);

            TrajectorySequence juncTraj2 = drive.trajectorySequenceBuilder(startPose)
                    .forward(4)               // goes forward
                    .strafeLeft(5)
                    .turn(toRadians(-90))
                    .strafeLeft(60)  // to high junction
                    .forward(2)
                    .build();
            drive.followTrajectorySequence(juncTraj2);
            while (drive.isBusy()) sleep(400);

            Slide.setPower(0.9);

            Slide.setPower(0.3);
            Slide.setTargetPosition(SLIDE_INITIAL + 2570);    //get cone in the high junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);
            Pinch.setPosition(0.14);        //release the cone on high junction
            sleep(110);

            TrajectorySequence juncTraj3 = drive.trajectorySequenceBuilder(juncTraj2.end())
                    .back(2)
                    .strafeRight(8)             //move forward to the high junction
                    .turn(toRadians(-180))
                    .build();
            drive.followTrajectorySequence(juncTraj3);
            while (drive.isBusy()) sleep(100);

            Slide.setTargetPosition(SLIDE_INITIAL + 390); //Makes sure to move to correct position
            Slide.setPower(0.9);
            Pinch.setPosition(0.14);
            sleep(110);

            TrajectorySequence juncTraj4 = drive.trajectorySequenceBuilder(juncTraj3.end())
                    .lineToSplineHeading(new Pose2d(63, 10.5, toRadians(0)))
                    .build();
            drive.followTrajectorySequence(juncTraj4);
            while (drive.isBusy()) sleep(300);

            Pinch.setPosition(0.7);      //pinch closes
            sleep(500);
            Slide.setTargetPosition(SLIDE_INITIAL + 900); //slide shifts up
            Slide.setPower(0.9);

            TrajectorySequence juncTraj8 = drive.trajectorySequenceBuilder(juncTraj4.end())
                    .back(13)         //moves away from the high junction
                    .build();
            drive.followTrajectorySequence(juncTraj8);
            while (drive.isBusy()) sleep(150);


            Slide.setTargetPosition(SLIDE_INITIAL +1030); //raises for medium junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);


            TrajectorySequence juncTraj9 = drive.trajectorySequenceBuilder(juncTraj8.end())
                    .turn(toRadians(90))   //turns to face medium junction
                    .forward(5) //moves forward to medium junction
                    .build();
            drive.followTrajectorySequence(juncTraj9);
            while (drive.isBusy()) sleep(100);

            Slide.setTargetPosition(SLIDE_INITIAL + 970); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(200);
            Pinch.setPosition(0.11); // releases cone on short junction
            sleep(200);

            TrajectorySequence juncTraj10 = drive.trajectorySequenceBuilder(juncTraj9.end())
                    .back(4) //returns to middle
                    .turn(toRadians(-90)) //rotates to face blue line again
                    .build();
            drive.followTrajectorySequence(juncTraj10);

            Slide.setTargetPosition(SLIDE_INITIAL + 355); //raises for 2nd Cone
            sleep(200);
            Pinch.setPosition(0.14);
            sleep(110);

            TrajectorySequence juncTraj11 = drive.trajectorySequenceBuilder(juncTraj10.end())
                    .lineToSplineHeading(new Pose2d(63, 10.5, toRadians(0)))
                    .build();
            drive.followTrajectorySequence(juncTraj11);
            while (drive.isBusy()) sleep(300);

            Pinch.setPosition(0.7);      //pinch closes
            sleep(500);
            Slide.setTargetPosition(SLIDE_INITIAL + 900); //slide shifts up
            Slide.setPower(0.9);

            TrajectorySequence juncTraj12 = drive.trajectorySequenceBuilder(juncTraj11.end())
                    .back(13)         //moves away from the high junction
                    .build();
            drive.followTrajectorySequence(juncTraj12);
            while (drive.isBusy()) sleep(150);


            Slide.setTargetPosition(SLIDE_INITIAL +1030); //raises for medium junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);


            TrajectorySequence juncTraj13 = drive.trajectorySequenceBuilder(juncTraj12.end())
                    .turn(toRadians(90))   //turns to face medium junction
                    .forward(5) //moves forward to medium junction
                    .build();
            drive.followTrajectorySequence(juncTraj13);
            while (drive.isBusy()) sleep(100);

            Slide.setTargetPosition(SLIDE_INITIAL + 970); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(200);
            Pinch.setPosition(0.11); // releases cone on short junction
            sleep(200);

            TrajectorySequence juncTraj14 = drive.trajectorySequenceBuilder(juncTraj13.end())
                    .back(4) //returns to middle
                    .turn(toRadians(-90)) //rotates to face blue line again
                    .build();
            drive.followTrajectorySequence(juncTraj14);

            TrajectorySequence parkingTraj;
            switch (visionResult) {
                case LEFT: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(35)
                            .build();
                    break;
                }
                case CENTER: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(11)
                            .build();
                    break;
                }
                default: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .back(13)
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