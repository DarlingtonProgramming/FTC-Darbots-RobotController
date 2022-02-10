package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.Chassis2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.DriveMethod;
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.FieldConstant;
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive_Chassis2;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.List;

import static java.lang.Math.toRadians;

@Autonomous(name = "BLUE DUCK - WAREHOUSE 2", group = "A Competition")
public class Mecanum_Auto_BlueDuck_Warehouse extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private CRServo Spin = null;
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    //Vuforia setup for vision
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            Robot4100Common.VUFORIA_LICENSE;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        PoseStorage.autoState = DriveMethod.poseState.BLUE;

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(CRServo.class, "Spin");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Spin.setDirection(DcMotor.Direction.FORWARD);


        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Rotate.setDirection(Servo.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize position
        Slide.setPower(0.15);
        sleep(100);
        Slide.setPower(0.0);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotate.setPosition(0.85);

        //Variables
        double center = -1;
        int initialHeight = Slide.getCurrentPosition();
        final int TARGET_HEIGHT;
        String visionResult = null;

        //Vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }

        //Traj
        SampleMecanumDrive_Chassis2 drive = new SampleMecanumDrive_Chassis2(hardwareMap);
        Pose2d startPose = FieldConstant.BLUE_DUCK_STARTING_POSE;
        drive.setPoseEstimate(startPose);
        Trajectory duckTraj = drive.trajectoryBuilder(startPose,true)
                .splineToLinearHeading(new Pose2d(-56.91, 56.91, toRadians(135)), toRadians(0))
                .build();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Object Recognition Starts
        ElapsedTime recogTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        int recogDuration = 1500;
        if (visionResult == null) {
            recogTime.reset();
        }
        while (!opModeIsActive()){
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

        waitForStart();
        recogDuration -= recogTime.milliseconds();
        if(opModeIsActive()) {
            runtime.reset();
            if(recogDuration > 1000){
                recogDuration = 1500;
            }
            else if (recogDuration > 500) {
                recogDuration = 1000;
            }
            else {
                recogDuration = 0;
            }
            telemetry.addData("recogDuration", recogDuration);
            telemetry.update();
            while (runtime.milliseconds() < recogDuration){
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

            if (center < 0) {
                visionResult = "LEFT";
                TARGET_HEIGHT = initialHeight;
            } else if (center < 273) {
                visionResult = "MIDDLE";
                TARGET_HEIGHT = initialHeight + 500;
            } else {
                visionResult = "RIGHT";
                TARGET_HEIGHT = initialHeight + 1150;
            }
            telemetry.addLine(visionResult);
            telemetry.update();

            //MOTION TO DUCK
            drive.followTrajectory(duckTraj);
            sleep(300);

            DriveMethod.spinDuck(drive, Spin, PoseStorage.autoState);

            Pose2d wall = new Pose2d(-56.91, 56.91, drive.getExternalHeading()); //Math.toRadians(90
            drive.setPoseEstimate(wall);

            //MOTION TO PLATE
            Trajectory wallTraj = drive.trajectoryBuilder(duckTraj.end(), true)
                    .splineToLinearHeading(new Pose2d(-59, 23.75, toRadians(180)), toRadians(-90))
                    .build();
            drive.followTrajectory(wallTraj);

            Trajectory plateTraj = drive.trajectoryBuilder(wallTraj.end())
                    .back(28)
                    .addDisplacementMarker(25, () -> {
                        DriveMethod.slideUp(Intake, Rotate, Slide, TARGET_HEIGHT);
                    })
                    .build();
            drive.followTrajectory(plateTraj);
            sleep(300);

            //DUMP AND SLIDE DOWN
            DriveMethod.dump(Rotate);
            sleep(200);


            //BACK TO WAREHOUSE & PARK
            Trajectory parkTraj1 = drive.trajectoryBuilder(plateTraj.end())
                    .strafeRight(29)
                    .addTemporalMarker(1, () -> {
                        DriveMethod.slideDown(Slide, Rotate, initialHeight);
                    })
                    .build();
            drive.followTrajectory(parkTraj1);
            Trajectory parkTraj2 = drive.trajectoryBuilder(parkTraj1.end())
                    .strafeLeft(6)
                    .build();
            drive.followTrajectory(parkTraj2);
            Trajectory parkTraj3 = drive.trajectoryBuilder(parkTraj2.end())
                    .lineToLinearHeading(new Pose2d(-31.625, 47.75, toRadians(0)))
                    .build();
            drive.followTrajectory(parkTraj3);
            Trajectory parkTraj4 = drive.trajectoryBuilder(parkTraj3.end())
                    .forward(78)
                    .build();
            drive.followTrajectory(parkTraj4);
            sleep(500);

            PoseStorage.currentPose = drive.getPoseEstimate();

        }

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
    private void rotateWithSpeed(double targetPos, double factor){
        //1 -> 10 0.5 -> 5
        if(factor == 1.0){
            Rotate.setPosition(targetPos);
        }
        double currentPos = Rotate.getPosition();
        double interval = 0.05 * factor;
        while (targetPos > Rotate.getPosition()){
            Rotate.setPosition(Rotate.getPosition() + interval);
            sleep(30);
        }
        while (targetPos < Rotate.getPosition()){
            Rotate.setPosition(Rotate.getPosition() - interval);
            sleep(30);
        }
    }

}