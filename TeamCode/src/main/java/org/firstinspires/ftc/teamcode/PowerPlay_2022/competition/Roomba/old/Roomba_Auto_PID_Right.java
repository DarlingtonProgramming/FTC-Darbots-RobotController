package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.old;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Settings.Roomba_Constants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.detection.PPDetector;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.detection.classification.Classifier;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.FieldConstant;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;

@Disabled
@Autonomous(name = "Roomba Auto (Right)", group = "Competition")
public class Roomba_Auto_PID_Right extends LinearOpMode {

    private DcMotor LF, RF, LB, RB, Slide;
    private CRServo Turn;
    private Servo Pinch;
    private double speed = Roomba_Constants.INITIAL_SPEED;

    // Sleeve detection setup
    private static String MODEL_FILE_NAME = "pp_model.tflite";
    private static String LABEL_FILE_NAME = "labels.txt";
    private static final String[] LABELS = {
            "0 bat",
            "1 lantern",
            "2 eyes"
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
        Pinch.setPosition(Roomba_Constants.PINCH_MAX);

        // Detection
        initDetection();

        //Variables
        String visionResult = null;

        // Initialize roadrunner
        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = FieldConstant.BLUE_RIGHT;
        drive.setPoseEstimate(startPose);

        while (!opModeIsActive()) {
                List<Classifier.Recognition> results = sleeveDetector.getLastResults();
                if (results == null || results.size() == 0){
                    telemetry.addData("Info", "No results");
                } else {
                    for (Classifier.Recognition r : results) {
                        if (r.getConfidence() > 0.6f) {
                            visionResult = r.getTitle();
                        }
                    }
                }
        }
        if (sleeveDetector != null){
            sleeveDetector.stopProcessing();
        }

        telemetry.addData(">", "Finished initialization.");
        telemetry.update();

        waitForStart();
        final int SLIDE_INITIAL = Slide.getCurrentPosition();
        if (opModeIsActive()) {
            while (visionResult == null) {
                List<Classifier.Recognition> results = sleeveDetector.getLastResults();
                if (results == null || results.size() == 0){
                    telemetry.addData("Info", "No results");
                } else  {
                    for (Classifier.Recognition r : results) {
                        if (r.getConfidence() > 0.6f) {
                            visionResult = r.getTitle();
                        }
                        telemetry.update();
                    }
                }
            }
            if (sleeveDetector != null){
                sleeveDetector.stopProcessing();
            }
            telemetry.addLine("Found: " + visionResult);
            telemetry.update();

            TrajectorySequence juncTraj = drive.trajectorySequenceBuilder(startPose)
                    .forward(2)
                    .strafeLeft(30)
                    .forward(50)
                    .turn(toRadians(-52.25))
                    .build();
            drive.followTrajectorySequence(juncTraj);
            while (drive.isBusy()) sleep(500);

            slideTo(SLIDE_INITIAL + Roomba_Constants.SL_HIGH, 0.8);
            sleep(1750);

            TrajectorySequence juncTraj3 = drive.trajectorySequenceBuilder(juncTraj.end())
                    .forward(3)
                    .build();
            drive.followTrajectorySequence(juncTraj3);
            while (drive.isBusy()) sleep(250);

            sleep(600);
            slideTo(Slide.getCurrentPosition() - 105, 0.8);
            sleep(600);
            setPinched(false);
            sleep(600);
            slideTo(Slide.getCurrentPosition() + 105, 0.8);
            sleep(600);

            TrajectorySequence juncTraj4 = drive.trajectorySequenceBuilder(juncTraj3.end())
                    .back(6)
                    .addDisplacementMarker(2, () -> {
                        slideTo(SLIDE_INITIAL, 0.8);
                    })
                    .waitSeconds(1)
                    .turn(toRadians(52.25))
                    .build();
            drive.followTrajectorySequence(juncTraj4);
            while (drive.isBusy()) sleep(300);

            if (visionResult.equals(LABELS[0])) {
                Trajectory juncTraj5 = drive.trajectoryBuilder(juncTraj4.end())
                        .strafeRight(44)
                        .build();
                drive.followTrajectory(juncTraj5);
            } else if (visionResult.equals(LABELS[2])) {
                Trajectory juncTraj5 = drive.trajectoryBuilder(juncTraj4.end())
                        .strafeRight(22)
                        .build();
                drive.followTrajectory(juncTraj5);
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

    private void slideTo(int targetPosition, double power) {
        Slide.setTargetPosition(targetPosition);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);
    }

    private void setPinched(boolean pinched) {
        if (pinched)
            Pinch.setPosition(Roomba_Constants.PINCH_MAX);
        else
            Pinch.setPosition(Roomba_Constants.PINCH_MIN);
        sleep(500);
    }
}