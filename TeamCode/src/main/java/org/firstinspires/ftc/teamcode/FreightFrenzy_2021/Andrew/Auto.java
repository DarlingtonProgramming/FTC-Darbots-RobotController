package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.Andrew;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

@TeleOp(name = "Auto", group = "Linear OpMode")
public class Auto extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Spin = null;
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private Servo Push = null;
    private ArrayList<Double[]> speedList = new ArrayList<Double[]>();
    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.5;
    boolean reverse = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide  = hardwareMap.get(DcMotor.class, "Slide");
        Intake  = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spin.setDirection(DcMotor.Direction.FORWARD);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Rotate.setDirection(Servo.Direction.FORWARD);
        Push = hardwareMap.get(Servo.class, "Push");


        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        double intakePower = 0;
        double spinPower = 0;
        boolean levelOne   = false;
        boolean levelTwo   = false;
        boolean levelThree = false;

        int initialHeight = Slide.getCurrentPosition();

        waitForStart();


        while (opModeIsActive()) {
            runtime.reset();

            if(levelOne) {
                Slide.setTargetPosition(initialHeight);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.6);
            }


            if(levelTwo){
                Slide.setTargetPosition(initialHeight + 750);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.6);

            }

            if(levelThree){
                Slide.setTargetPosition(initialHeight + 1400);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.6);

            }

            if (true) {
                LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addLine("BREAK");
            } else {
                LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                telemetry.addLine("FLOAT");
            }

            if (true) {
                intakePower = 0.8;
                telemetry.addLine("INTAKE STARTS");
            }else{
                intakePower = 0;
                telemetry.addLine("INTAKE STOPS");
            }

            if(true){
                spinPower = 0.5;
                telemetry.addLine("SPIN STARTS");
            } else {
                spinPower = 0;
                telemetry.addLine("SPIN STOPS");
            }




            LFPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate - strafe), -1.0, 1.0) ;


            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);
            Intake.setPower(intakePower);
            Spin.setPower(spinPower);

            telemetry.addData("Servo","Rotate (%.2f), Push (%.2f)", Rotate.getPosition(), Push.getPosition());
            telemetry.addLine("Intake: " + intakePower);
            telemetry.addLine("Spin: " + spinPower);
            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("Speed:", speed);

            telemetry.update();

        }
    }

    private void decreaseSpeed(double s) {
        double decreased = speed - s;
        if (decreased < 0) {
            speed = 0;
            return;
        }
        speed = decreased;
    }

    private void increaseSpeed(double s) {
        double increased = speed + s;
        if (1 < increased) {
            speed = 1;
            return;
        }
        speed = increased;
    }
}