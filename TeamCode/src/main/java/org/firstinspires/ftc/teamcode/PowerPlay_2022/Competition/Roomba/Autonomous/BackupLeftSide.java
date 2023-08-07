package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Autonomous;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.Color.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.RoombaDriveMethod;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.DriveConstants_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Kimetsu no Yaiba the backup \uD83D\uDE08 \uD83D\uDE08 \uD83D\uDE08frfr!", group = "2")
public class BackupLeftSide extends LinearOpMode {

    private DistanceSensor Distance;
    private DcMotor Slide;
    private Servo Pinch;
    private WebcamName Webcam;
    private OpenCvCamera Camera;
    private ColorDetectionPipeline SleeveDetector;

    @Override
    public void runOpMode() {
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
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
        //62
        drive.setPoseEstimate(startPose);

        RoombaDriveMethod driveMethod = new RoombaDriveMethod(drive, Slide, Pinch);

        final int SLIDE_INITIAL = Slide.getCurrentPosition();
        Pinch.setPosition(0.9);

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
            //1250
            Slide.setTargetPosition(SLIDE_INITIAL + 2000);  //slide sets 2750 moves to high junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.8);
            sleep(50);

            //Move to high junction
            TrajectorySequence juncTraj2 = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(startPose.getX() + 1.5, startPose.getY() - 3))
                    .lineToSplineHeading(new Pose2d(39,24,toRadians(180)))
                    .forward(6)
                    .build();
            drive.followTrajectorySequence(juncTraj2);
            while (drive.isBusy()) sleep(90);

            Slide.setPower(0.9);

            Slide.setPower(0.4);
            Slide.setTargetPosition(SLIDE_INITIAL + 1700);    //get cone in the high junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(50);
            Pinch.setPosition(0.14);        //release the cone on high junction
            sleep(60);

            Pose2d check = drive.getPoseEstimate();

            TrajectorySequence juncTraj3 = drive.trajectorySequenceBuilder(juncTraj2.end())
                    .addDisplacementMarker(3, () -> {
                        Pinch.setPosition(0.14);
                    })
                    .addDisplacementMarker(5, () -> {
                        Slide.setTargetPosition(SLIDE_INITIAL + 450); //Makes sure to move to correct position
                        Slide.setPower(0.9);
                    })
                    .back(5)
                    .strafeLeft(13)
                    //.lineToSplineHeading(new Pose2d(check.getX(),check.getY() - 12, toRadians(0)))
                    .turn(toRadians(-180))
                    //.strafeLeft(15)
                    .build();
            drive.followTrajectorySequence(juncTraj3);

            sleep(70);

            TrajectorySequence juncTraj4 = drive.trajectorySequenceBuilder(juncTraj3.end())
                    //.turn(toRadians(-180))
                    //.lineToSplineHeading(new Pose2d(55,10.6,toRadians(0)))
                    .forward(14)
                    .build();                        //60.8        //10.6
            drive.followTrajectorySequence(juncTraj4);
            while (drive.isBusy()) sleep(300);

            do {
                if (Distance.getDistance(DistanceUnit.MM) > 90) {
                    drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
                } else {
                    drive.setMotorPowers(0.1, 0.1, 0.1, 0.1);
                }
            } while (Distance.getDistance(DistanceUnit.MM) > 40);

            drive.setMotorPowers(0, 0, 0, 0);

            Pinch.setPosition(0.9);
            sleep(500);
            Slide.setTargetPosition(SLIDE_INITIAL + 900); //slide shifts up
            Slide.setPower(0.9);


            //2030
            Slide.setTargetPosition(SLIDE_INITIAL + 2000); //raises for low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);


            TrajectorySequence juncTraj9 = drive.trajectorySequenceBuilder(juncTraj4.end())
                    .back(3.5)         //moves away from the high junction
                    .lineToSplineHeading(
                            new Pose2d(23,13,toRadians(90)),
                            MecanumDrive_Roomba.getVelocityConstraint(45, DriveConstants_Roomba.MAX_ANG_VEL, DriveConstants_Roomba.TRACK_WIDTH),
                            MecanumDrive_Roomba.getAccelerationConstraint(45)
                    )
                    /*                                         //.4
                    .turn(toRadians(90))   //turns to face high junction
                    .forward(5) //moves forward to medium junction
                     */
                    .forward(3.5)
                    .build();
            drive.followTrajectorySequence(juncTraj9);
            while (drive.isBusy()) sleep(100);

            //1800
            Slide.setTargetPosition(SLIDE_INITIAL + 1700); //lowers the cone on the low junction
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

            TrajectorySequence juncTraj11 = drive.trajectorySequenceBuilder(juncTraj10.end())
                    //   .forward(15.3)           //goes forward to cones for second cone
                    //10.5
                    .forward(25)
                    //     .lineToSplineHeading(new Pose2d(-67,-7.5,toRadians(180)))
                    .build();
            drive.followTrajectorySequence(juncTraj11);
            while (drive.isBusy()) sleep(150);

            do {
                if (Distance.getDistance(DistanceUnit.MM) > 90) {
                    drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
                } else {
                    drive.setMotorPowers(0.1, 0.1, 0.1, 0.1);
                }
            } while (Distance.getDistance(DistanceUnit.MM) > 40);

            drive.setMotorPowers(0, 0, 0, 0);


            Pinch.setPosition(0.9);
            sleep(500);
            Slide.setTargetPosition(SLIDE_INITIAL + 900); //slide shifts up
            Slide.setPower(0.9);

            //2030
            Slide.setTargetPosition(SLIDE_INITIAL + 2000); //raises for low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);

            TrajectorySequence juncTraj13 = drive.trajectorySequenceBuilder(juncTraj11.end())
                    .back(3.5)         //moves away from the high junction
                    .lineToSplineHeading(
                            new Pose2d(23,13,toRadians(90)),
                            MecanumDrive_Roomba.getVelocityConstraint(45, DriveConstants_Roomba.MAX_ANG_VEL, DriveConstants_Roomba.TRACK_WIDTH),
                            MecanumDrive_Roomba.getAccelerationConstraint(45)
                    )
                    /*                                         //.4
                    .turn(toRadians(90))   //turns to face high junction
                    .forward(5) //moves forward to medium junction
                     */
                    .forward(3.5)
                    .build();
            drive.followTrajectorySequence(juncTraj13);
            while (drive.isBusy()) sleep(100);
            //1800
            Slide.setTargetPosition(SLIDE_INITIAL + 1700); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.7);

            sleep(200);
            Pinch.setPosition(0.1); // releases cone on short junction
            sleep(300);

            TrajectorySequence juncTraj14 = drive.trajectorySequenceBuilder(juncTraj13.end())
                    .back(3.5) //returns to middle
                    //.turn(toRadians(-90)) //rotates to face blue line again
                    .build();
            drive.followTrajectorySequence(juncTraj14);
            while (drive.isBusy()) sleep(100);

            /*
            Slide.setTargetPosition(SLIDE_INITIAL + 210); //raises for 2nd Cone
            sleep(200);

            TrajectorySequence juncTraj15 = drive.trajectorySequenceBuilder(juncTraj14.end())
                    .lineToSplineHeading(new Pose2d(60.8,12,toRadians(0)))
                    .build();
            drive.followTrajectorySequence(juncTraj15);

            Slide.setPower(0.9);
            Pinch.setPosition(0.9);      //pinch closes on second cone
            sleep(300);
            Slide.setTargetPosition(SLIDE_INITIAL +950); //slide shifts up
            Slide.setPower(0.9);

            TrajectorySequence juncTraj16 = drive.trajectorySequenceBuilder(juncTraj15.end())
                    .back(3)           //backs up from cones
                    .build();
            drive.followTrajectorySequence(juncTraj16);
            while (drive.isBusy()) sleep(300);
            //2030
            Slide.setTargetPosition(SLIDE_INITIAL +1230); //raises for short junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);

            TrajectorySequence juncTraj17 = drive.trajectorySequenceBuilder(juncTraj16.end())
                    .lineToSplineHeading(new Pose2d(46.5,15.5,toRadians(89)))
                    .build();
            drive.followTrajectorySequence(juncTraj17);
            while (drive.isBusy()) sleep(100);
            //1800
            Slide.setTargetPosition(SLIDE_INITIAL + 980); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(200);
            Pinch.setPosition(0.1); // releases cone on short junction
            sleep(300);
            */

            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setTargetPosition(SLIDE_INITIAL);
            Slide.setPower(0.3);
            sleep(100);

            TrajectorySequence parkingTraj;
            switch (visionResult) {
                case LEFT: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //.back(4)
                            .strafeRight(34)
                            .build();
                    drive.followTrajectorySequence(parkingTraj);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    break;
                }
                case CENTER: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //.back(4)
                            .strafeRight(12)
                            .build();
                    drive.followTrajectorySequence(parkingTraj);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    break;
                }
                default: {
                    parkingTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            //.back(4)
                            .strafeLeft(10)
                            // .back(4)
                            // .strafeLeft(18)
                            .build();
                    drive.followTrajectorySequence(parkingTraj);
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    break;
                }
            }
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