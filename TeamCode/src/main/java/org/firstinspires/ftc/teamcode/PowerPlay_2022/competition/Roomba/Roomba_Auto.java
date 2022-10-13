package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.List;

import static java.lang.Math.toRadians;

@Autonomous(name = "Roomba - Auto", group = "Competition")
public class Roomba_Auto extends LinearOpMode {

    private DcMotor LF, RF, LB, RB, Slide;
    private CRServo Turn;
    private Servo Pinch;

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

    double rotate = 0;
    boolean reverse = false;

    @Override
    public void runOpMode() {
        /*
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
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);

        // Use slide positions
        final int SLIDE_INITIAL = Slide.getCurrentPosition();
        int slideStoppedPos = SLIDE_INITIAL;

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize position
        Push.setPosition(0.4);
        Slide.setPower(0.15);
        sleep(100);
        Slide.setPower(0.0);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotate.setPosition(0.03);

        //Vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        //Variables
        double center = -1;
        int initialHeight = Slide.getCurrentPosition();
        String visionResult = null;
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //TRAJ
        SampleMecanumDrive_Chassis1 drive = new SampleMecanumDrive_Chassis1(hardwareMap);

        waitForStart();
        if(opModeIsActive()) {

            ElapsedTime recogTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            if (visionResult == null) {
                recogTime.reset();
            }

            while (recogTime.milliseconds() <= 2000.0) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            if (recognition.getLabel().equals("Duck")) {
                                center = (recognition.getLeft() + recognition.getRight()) / 2.0;
                            }
                            i++;
                        }
                        telemetry.update();
                    }
                }
            }
            Pose2d startPose = FieldConstant.BLUE_BARRIER_STARTING_POSE;
            drive.setPoseEstimate(startPose);
            if (center < 0) {
                visionResult = "LEFT";
            } else if (center < 321.85) {
                visionResult = "MIDDLE";
                Trajectory plateTraj0 = drive.trajectoryBuilder(startPose,true)
                        .strafeRight(5)
                        .build();
                drive.followTrajectory(plateTraj0);
                startPose = plateTraj0.end();
            } else {
                visionResult = "RIGHT";
                Trajectory plateTraj0 = drive.trajectoryBuilder(startPose,true)
                        .strafeRight(5)
                        .build();
                drive.followTrajectory(plateTraj0);
                startPose = plateTraj0.end();
            }
            telemetry.addLine(visionResult);
            telemetry.update();

            //MOVE MARKER OUT
            Trajectory plateTraj1 = drive.trajectoryBuilder(startPose,true)
                    .back(45)
                    .build();
            drive.followTrajectory(plateTraj1);
            Trajectory plateTraj2 = drive.trajectoryBuilder(plateTraj1.end())
                    .forward(2)
                    .build();
            drive.followTrajectory(plateTraj2);

            //MOTION TO PLATE
            Trajectory plateTraj3 = drive.trajectoryBuilder(plateTraj2.end())
                    .lineToLinearHeading(new Pose2d(7.875, 23.75, toRadians(0)))
                    .build();
            drive.followTrajectory(plateTraj3);

            //ROTATE
            Rotate.setPosition(1.0);
            sleep(800);

            //SLIDE UP
            if (visionResult == "LEFT") {
                Slide.setTargetPosition(initialHeight);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            } else if (visionResult == "MIDDLE") {
                Slide.setTargetPosition(initialHeight + 700);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            } else if (visionResult == "RIGHT") {
                Slide.setTargetPosition(initialHeight + 1500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            }
            sleep(600);

            //CLOSER TO PLATE
            Trajectory closerTraj = drive.trajectoryBuilder(plateTraj3.end())
                    .back(1.5)
                    .build();
            drive.followTrajectory(closerTraj);
            drive.setPoseEstimate(closerTraj.end());

            //DUMP AND SLIDE DOWN
            Push.setPosition(0.0);
            sleep(300);
            Push.setPosition(0.4);
            sleep(500);
            Rotate.setPosition(0.03);
            sleep(500);
            Slide.setTargetPosition(initialHeight);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.8);

            //BACK TO WALL
            if(visionResult == "RIGHT"){
                Trajectory wallTrajR1 = drive.trajectoryBuilder(closerTraj.end())
                        .lineToLinearHeading(new Pose2d(14.8, 23.75, toRadians(90)))
                        .build();
                drive.followTrajectory(wallTrajR1);
                Trajectory wallTrajR2 = drive.trajectoryBuilder(wallTrajR1.end())
                        .forward(20)
                        .build();
                drive.followTrajectory(wallTrajR2);
                drive.setPoseEstimate(wallTrajR2.end());
            }
            Trajectory wallTraj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(6.5, 65, toRadians(0)))
                    .build();
            drive.followTrajectory(wallTraj1);
            sleep(300);

            Trajectory wallTraj2 = drive.trajectoryBuilder(wallTraj1.end())
                    .forward(36)
                    .build();
            drive.followTrajectory(wallTraj2);


            PoseStorage.currentPose = wallTraj1.end();
            PoseStorage.state = DriveMethod.poseState.BLUE;
        }
        */

    }

    //Vision
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}