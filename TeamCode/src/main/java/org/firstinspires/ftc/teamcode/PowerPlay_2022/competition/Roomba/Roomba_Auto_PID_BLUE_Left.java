package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.FieldConstant;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.List;

@Autonomous(name = "Roomba Auto (BLUE - Left)", group = "Competition")
public class Roomba_Auto_PID_BLUE_Left extends LinearOpMode {

    private DcMotor LF, RF, LB, RB, Slide;
    private CRServo Turn;
    private Servo Pinch;
    private WebcamName Webcam;

    private double speed = Roomba_Constants.INITIAL_SPEED;

    //Vuforia setup for vision
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY = Robot4100Common.VUFORIA_LICENSE;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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

        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

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

        //initialize position
        Pinch.setPosition(Roomba_Constants.PINCH_MAX);

        //Vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        //Variables
        final int SLIDE_INITIAL = Slide.getCurrentPosition();
        final Pose2d END_POS;
        String visionResult = null;

        // Initialize roadrunner
        MecanumDrive_Roomba drive = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = new Pose2d(35, 60, toRadians(180));
        drive.setPoseEstimate(startPose);

        // Build trajectory to medium junction
        ElapsedTime recogTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        recogTime.reset();
        while (!opModeIsActive()) {
            while (recogTime.milliseconds() <= 2000.0) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            visionResult = recognition.getLabel();
                        }
                        telemetry.update();
                    }
                }
            }
        }

        telemetry.addData(">", "Initialized.");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            if (visionResult == null) {
                recogTime.reset();
            }
            while (recogTime.milliseconds() <= 2000.0) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            visionResult = recognition.getLabel();
                        }
                        telemetry.update();
                    }
                }
            }

            while (visionResult == null) {
                sleep(1000);
                telemetry.addLine("waitn");
                telemetry.update();
            }

            if (visionResult.equals(LABELS[0])) { // Bolt
                END_POS = FieldConstant.BL_BOLT_PARK;
            } else if (visionResult.equals(LABELS[1])) { // Bulb
                END_POS = FieldConstant.BL_BULB_PARK;
            } else { // Assume panel
                END_POS = FieldConstant.BL_PANEL_PARK;
            }

            telemetry.addLine(visionResult);
            telemetry.update();

            Trajectory juncTraj = drive.trajectoryBuilder(startPose, true)
                    .forward(20)
                    /*
                    .addDisplacementMarker(30, () -> {
                        slideTo(SLIDE_INITIAL + Roomba_Constants.SL_MEDIUM, 0.8);
                    })

                     */
                    .build();
            /*
            Trajectory juncTraj2 = drive.trajectoryBuilder(juncTraj.end())
                    .lineToSplineHeading(new Pose2d(10, 34, toRadians(225)))
                    .build();

             */

            drive.followTrajectory(juncTraj);
            //drive.followTrajectory(juncTraj2);

            /*
            // Motions to junction
            Trajectory juncTraj3 = drive.trajectoryBuilder(juncTraj2.end(),true)
                    .forward(5)
                    .addDisplacementMarker(2, () -> {
                        slideTo(SLIDE_INITIAL + Roomba_Constants.SL_HIGH, 0.8);
                    })
                    .build();
            while (drive.isBusy()) sleep(250);
            drive.followTrajectory(juncTraj3);
            setPinched(false);
            Trajectory juncTraj4 = drive.trajectoryBuilder(juncTraj3.end())
                    .back(5)
                    .addTemporalMarker(0.75, () -> {
                        slideTo(SLIDE_INITIAL, 0.8);
                    })
                    .build();
            while (drive.isBusy()) sleep(250);
            drive.followTrajectory(juncTraj4);
            Trajectory juncTraj5 = drive.trajectoryBuilder(juncTraj4.end())
                    .lineToSplineHeading(new Pose2d(12, 12, toRadians(180)))
                    .build();
            while (drive.isBusy()) sleep(250);
            drive.followTrajectory(juncTraj5);
            Trajectory parkTraj = drive.trajectoryBuilder(juncTraj5.end())
                    .lineToSplineHeading(END_POS)
                    .build();
            while (drive.isBusy()) sleep(250);
            drive.followTrajectory(parkTraj);


             */
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
        while (opModeIsActive()) {
            Pose2d currentPose = drive.getPoseEstimate();
            telemetry.addLine("x: " + currentPose.getX());
            telemetry.addLine("y: " + currentPose.getY());
            telemetry.update();
        }
    }

    // Vision
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = Webcam;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // Object Detection
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.40f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
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