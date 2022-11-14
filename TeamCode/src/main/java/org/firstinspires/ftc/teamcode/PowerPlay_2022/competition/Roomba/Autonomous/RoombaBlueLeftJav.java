package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Autonomous;

import static java.lang.Math.toRadians;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.detection.PPDetector;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.detection.classification.Classifier;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "[2S 1H] Roomba Blue Left", group = "Circuit Blue")
public class RoombaBlueLeftJav extends LinearOpMode {
    private DcMotor LF, RF, LB, RB, Slide;
    private CRServo Turn;
    private Servo Pinch;
    private double speed = 0.5;

    // Sleeve detection setup
    private static String MODEL_FILE_NAME = "Blue.tflite";
    private static String LABEL_FILE_NAME = "labels.txt";
    private static final String[] LABELS = {
            "0 eyes",   //for the parking eyes = 2
            "1 bat",    //bat = 3
            "2 lantern" //lantern = 1
    };
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
    private PPDetector sleeveDetector = null;
    @Override
    public void runOpMode() {
        // Get devices from hardware map
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "Slide");

        Turn = hardwareMap.get(CRServo.class, "Turn");
        Pinch = hardwareMap.get(Servo.class, "Pinch");

        // Initialize devices
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize position
        Pinch.setPosition(0.5);

        // Detection
        initDetection();
        final int SLIDE_INITIAL = Slide.getCurrentPosition();

        //Variables
        String visionResult = null;

        // Initialize roadrunner
        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = new Pose2d(30.5, 62, toRadians(270));
        drive.setPoseEstimate(startPose);

        while (!opModeIsActive()) {
            List<Classifier.Recognition> results = sleeveDetector.getLastResults();
            if (results == null || results.size() == 0){
                telemetry.addData("Info", "No results");
            } else {
                for (Classifier.Recognition r : results) {
                    if (r.getConfidence() > 0.5f) {
                        visionResult = r.getTitle();
                        telemetry.addLine("found: " + visionResult);
                        telemetry.update();
                    }
                }
            }
        }
        if (sleeveDetector != null){
            sleeveDetector.stopProcessing();
        }

        waitForStart();
        if (opModeIsActive()) {
            Pinch.setPosition(0.6);
            do {
                List<Classifier.Recognition> results = sleeveDetector.getLastResults();
                if (results != null && results.size() != 0){
                    for (Classifier.Recognition r : results) {
                        if (r.getConfidence() > 0.5f) {
                            visionResult = r.getTitle();
                        }
                    }
                }
            } while (visionResult == null && !isStopRequested());
            if (sleeveDetector != null) {
                sleeveDetector.stopProcessing();
            }
            telemetry.addLine("Found: " + visionResult);
            telemetry.update();

            Slide.setTargetPosition(SLIDE_INITIAL + 2750);  //slide sets 2750 moves to high junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.6);
            sleep(50);

            TrajectorySequence junc = drive.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(12, 58, toRadians(270)))
                    .lineToSplineHeading(new Pose2d(6, 27, toRadians(225)))
                    .build();
            drive.followTrajectorySequence(junc);
            while (drive.isBusy()) sleep(40);

            Slide.setPower(0.9);

            Slide.setTargetPosition(SLIDE_INITIAL + 2410);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.2);
            sleep(500);
            Pinch.setPosition(0.12);        //release the cone on high junction
            sleep(510);

            TrajectorySequence junc2 = drive.trajectorySequenceBuilder(junc.end())
                    .back(3)
                    .addDisplacementMarker(() -> {
                        Slide.setTargetPosition(SLIDE_INITIAL + (int) 505.10);    //slide sets 515
                        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Slide.setPower(0.7);
                        Pinch.setPosition(0.13);
                    })
                    .lineToSplineHeading(new Pose2d(11,32, toRadians(0)))
                    .lineToSplineHeading(new Pose2d(11,11.5, toRadians(0)))
                    .lineToSplineHeading(new Pose2d(63, 10.1, toRadians(0)))
                    .build();
            drive.followTrajectorySequence(junc2);
            while (drive.isBusy()) sleep(250);

            Slide.setTargetPosition(SLIDE_INITIAL + 420); //Makes sure to move to correct position
            Slide.setPower(0.9);
            Pinch.setPosition(0.7);      //pinch closes
            sleep(750);
            Slide.setTargetPosition(SLIDE_INITIAL + 900); //slide shifts up
            Slide.setPower(0.9);

            TrajectorySequence juncTraj8 = drive.trajectorySequenceBuilder(junc2.end())
                    .back(15)           //backs up from cones
                    .build();
            drive.followTrajectorySequence(juncTraj8);
            while (drive.isBusy()) sleep(300);

            Slide.setTargetPosition(SLIDE_INITIAL +1230); //raises for short junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);

            TrajectorySequence juncTraj9 = drive.trajectorySequenceBuilder(juncTraj8.end())
                    .turn(toRadians(90))   //turns to face short junction
                    .forward(4) //moves forward to short junction
                    .build();
            drive.followTrajectorySequence(juncTraj9);
            while (drive.isBusy()) sleep(100);

            Slide.setTargetPosition(SLIDE_INITIAL + 1000); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(200);
            Pinch.setPosition(0.11); // releases cone on short junction
            sleep(300);

            TrajectorySequence juncTraj10 = drive.trajectorySequenceBuilder(juncTraj9.end())
                    .back(4) //returns to middle
                    .turn(toRadians(-90)) //rotates to face blue line again
                    .build();
            drive.followTrajectorySequence(juncTraj10);

            Slide.setTargetPosition(SLIDE_INITIAL + 330); //raises for 2nd Cone
            sleep(200);

            TrajectorySequence juncTraj11 = drive.trajectorySequenceBuilder(juncTraj10.end())
                    //   .forward(15.3)           //goes forward to cones for second cone
                    .lineToSplineHeading(new Pose2d(63,10.1,toRadians(0)))
                    //     .lineToSplineHeading(new Pose2d(-67,-7.5,toRadians(180)))
                    .build();
            drive.followTrajectorySequence(juncTraj11);
            while (drive.isBusy()) sleep(150);

            Slide.setPower(0.9);
            Pinch.setPosition(0.7);      //pinch closes on second cone
            sleep(650);
            Slide.setTargetPosition(SLIDE_INITIAL +950); //slide shifts up
            Slide.setPower(0.9);

            TrajectorySequence juncTraj12 = drive.trajectorySequenceBuilder(juncTraj11.end())
                    .back(15)           //backs up from cones
                    .build();
            drive.followTrajectorySequence(juncTraj12);
            while (drive.isBusy()) sleep(300);

            Slide.setTargetPosition(SLIDE_INITIAL +1230); //raises for short junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);

            TrajectorySequence juncTraj13 = drive.trajectorySequenceBuilder(juncTraj12.end())
                    .turn(toRadians(90))   //turns to face short junction
                    .forward(4) //moves forward to short junction
                    .build();
            drive.followTrajectorySequence(juncTraj13);
            while (drive.isBusy()) sleep(100);

            Slide.setTargetPosition(SLIDE_INITIAL + 1000); //lowers the cone on the low junction
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.9);
            sleep(200);
            Pinch.setPosition(0.1); // releases cone on short junction
            sleep(300);

            TrajectorySequence juncTraj14 = drive.trajectorySequenceBuilder(juncTraj13.end())
                    .back(4) //returns to middle
                    .turn(toRadians(-90)) //rotates to face blue line again
                    .build();
            drive.followTrajectorySequence(juncTraj14);

            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setTargetPosition(SLIDE_INITIAL);
            Slide.setPower(0.9);
            sleep(100);

            if (visionResult.equals(LABELS[0])) { //if it equals eyes (park at 2)
                Trajectory juncTraj15 = drive.trajectoryBuilder(juncTraj14.end())
                        .back(14)
                        .build();
                drive.followTrajectory(juncTraj15);
            } else if (visionResult.equals(LABELS[2])) { //if it equals lantern (park at 1)
                Trajectory juncTraj15 = drive.trajectoryBuilder(juncTraj14.end())
                        .forward(12)
                        .build();
                drive.followTrajectory(juncTraj15);
            } else if (visionResult.equals(LABELS[1])) { //if it equals bat (park at 3)
                Trajectory juncTraj15 = drive.trajectoryBuilder(juncTraj14.end())
                        .back(10)
                        .build();
                drive.followTrajectory(juncTraj15);
            }
        }
    }

    private void initDetection() {
        try {
            try {
                sleeveDetector = new PPDetector(MODEl_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
                sleeveDetector.activate();
            } catch (Exception ex) {
                telemetry.addData("Error", String.format("Unable to initialize sleeve detector. %s", ex.getMessage()));
            }
        } finally {

        }
    }

}


