package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Settings.Roomba_Constants;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.List;

@Disabled
@Autonomous(name = "Roomba Auto (ALL)", group = "Competition")
public class Roomba_Auto_Motor extends LinearOpMode {

    private DcMotor LF, RF, LB, RB, Slide;
    private CRServo Turn;
    private Servo Pinch;
    private WebcamName Webcam;

    private final double POWER = 0.8;

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
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

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
            }

            telemetry.addLine(visionResult);
            telemetry.update();

            driveStraight(false, 1000);
            sleep(500);
            strafe(false, 250);

            strafe(false, 500);
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

    private void driveStraight(boolean reversed, double time) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (driveTime.milliseconds() < time) {
            if (reversed) {
                LF.setPower(-POWER);
                LB.setPower(-POWER);
                RB.setPower(-POWER);
                RF.setPower(-POWER);
            } else {
                LF.setPower(POWER);
                LB.setPower(POWER);
                RB.setPower(POWER);
                RF.setPower(POWER);
            }
        }
        stopDrive();
    }

    private void strafe(boolean left, double time) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (driveTime.milliseconds() < time) {
            if (!left) {
                LF.setPower(POWER);
                LB.setPower(-POWER);
                RB.setPower(POWER);
                RF.setPower(-POWER);
            } else {
                LF.setPower(-POWER);
                LB.setPower(POWER);
                RB.setPower(-POWER);
                RF.setPower(POWER);
            }
        }
        stopDrive();
    }

    private void stopDrive() {
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
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