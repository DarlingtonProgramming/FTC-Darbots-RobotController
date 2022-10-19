package org.firstinspires.ftc.teamcode.PowerPlay_2022.Test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.FieldConstant;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Roomba_Constants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.List;

public class Auto extends LinearOpMode {
    private DcMotor LF, RF, LB, RB, Slide;
    private CRServo Turn;
    private Servo Pinch;
    private WebcamName Webcam;
    private double speed = Roomba_Constants.INITIAL_SPEED;

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

        // Use slide positions
        final int SLIDE_INITIAL = Slide.getCurrentPosition();

        MecanumDrive_Roomba Roomba = new MecanumDrive_Roomba(hardwareMap);
        Pose2d startPose = FieldConstant.BLUE_LEFT;
        Roomba.setPoseEstimate(startPose);



        // Initialize pinch position
        Pinch.setPosition(Roomba_Constants.PINCH_MIN);

        //Vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        //Variables
        String visionResult = null;

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
        // Update telemetry
        telemetry.addData("Status", "Initialized");
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




            forward(3000, 0.8, true);
            left(3000,0.5,false);
            sleep(3000);


           /* if(visionResult == LABELS[0]){
                left(3000,0.5,true);
                forward(3000,0.5,false);
            } else if(visionResult == LABELS[1]){
                left(2000,0.5,true);
                forward(3000,0.5,false);
            }else{
                forward(3000,0.5,false);
            }
*/

            }
        }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = Webcam;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
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

    public void left(double time, double power, boolean t) {
        if (t) {
            LF.setPower(-power);
            RF.setPower(power);
            LB.setPower(-power);
            RB.setPower(power);
        }
        if (!t) {
            LF.setPower(power);
            RF.setPower(-power);
            LB.setPower(power);
            RB.setPower(-power);
        }
    }

    public void forward(double time, double power, boolean t) {
        if (t) {
            LF.setPower(power);
            RF.setPower(power);
            LB.setPower(power);
            RB.setPower(power);
        }
        if (!t) {
            LF.setPower(-power);
            RF.setPower(-power);
            LB.setPower(-power);
            RB.setPower(-power);
        }
    }
}






