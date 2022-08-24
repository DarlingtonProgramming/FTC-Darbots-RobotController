package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "FF - 1 CONTROLLER", group = "A Competition")
public class Mecanum_TeleOp_OneController_Old extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Slide = null;
    private CRServo Spin = null;
    private Servo Rotate = null;
    private Servo Arm = null;

    private DistanceSensor RangeSensor = null;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime blockTime = new ElapsedTime();

    //Pose2d lastPos = null;

    double speed = 0.6;

    @Override
    public void runOpMode() {
        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide  = hardwareMap.get(DcMotor.class, "Slide");
        Intake  = hardwareMap.get(DcMotor.class, "Intake");

        Spin = hardwareMap.get(CRServo.class, "Spin");
        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Arm = hardwareMap.get(Servo.class, "Arm");

        RangeSensor = hardwareMap.get(DistanceSensor.class, "Range");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Spin.setDirection(CRServo.Direction.FORWARD);
        Rotate.setDirection(Servo.Direction.FORWARD);
        Arm.setDirection(Servo.Direction.FORWARD);

        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        double intakePower = 0;
        double spinPower = 0;
        int initialHeight = Slide.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Rotate to initial position
        rotateWithSpeed(Rotate,0.95, 1);

        waitForStart();

        boolean releasedRB = true;
        boolean releasedLB = true;
        boolean releasedRT = true;
        boolean releasedA = true;
        boolean releasedB = true;
        boolean releasedY = true;
        boolean releasedDD = true;
        boolean releasedDU = true;
        boolean releasedLT = true;

        boolean releasedStart = true;
        boolean releasedRS = true;
        boolean releasedLS = true;

        boolean releasedDR = true;
        boolean releasedDL = true;

        boolean releasedBack = true;
        boolean toggleRT = true;
        boolean toggleLT = true;

        double rotateSpeed = 1;

        while (opModeIsActive()) {
            runtime.reset();
            if (gamepad1.x) {
                gamepad1.rumble(1000000);
            }

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            //////////////GAMEPAD 1//////////////

            if(gamepad1.dpad_right) {
                if(releasedDR) {
                    increaseSpeed(0.05);
                    releasedDR = false;
                }
            } else if(!releasedDR){
                releasedDR = true;
            }

            if(gamepad1.dpad_left){
                if(releasedDL) {
                    decreaseSpeed(0.05);
                    releasedDL = false;
                }
            } else if (!releasedDL){
                releasedDL = true;
            }
            if (gamepad1.left_bumper) {
                if (releasedLB && Slide.getCurrentPosition() < initialHeight + 30){
                    intakePower = 1;
                    telemetry.addLine("INTAKE STARTS");
                    releasedLB = false;
                }
            } else if (!releasedLB){
                intakePower = 0;
                telemetry.addLine("INTAKE STOPS");
                releasedLB = true;
            }
            if (gamepad1.right_bumper) {
                if (releasedRB){
                    intakePower = -1;
                    telemetry.addLine("INTAKE REVERSE STARTS");
                    releasedRB = false;
                }
            } else if (!releasedRB){
                telemetry.addLine("INTAKE STOPS");
                intakePower = 0;
                releasedRB = true;
            }

            if(gamepad1.left_stick_button){
                if(releasedLS) {
                    speed = 0.25;
                    releasedLS = false;
                }
            } else if(!releasedLS){
                releasedLS = true;
            }

            if(gamepad1.right_stick_button){
                if(releasedRS) {
                    speed = 0.6;
                    releasedRS = false;
                }
            } else if(!releasedRS){
                releasedRS = true;
            }

            if(gamepad1.a && !gamepad1.start && !gamepad2.start){
                if(releasedA) {
                    LF.setPower(0);
                    RF.setPower(0);
                    LB.setPower(0);
                    RB.setPower(0);
                    rotateWithSpeed(Rotate,0.90, rotateSpeed);
                    sleep(500);
                    Slide.setTargetPosition(initialHeight);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    Rotate.setPosition(0.95);
                    releasedA = false;
                }

            } else if(!releasedA){
                releasedA = true;
            }
            if(gamepad1.b && !gamepad1.start && !gamepad2.start){
                if(releasedB) {
                    LF.setPower(0);
                    RF.setPower(0);
                    LB.setPower(0);
                    RB.setPower(0);
                    rotateWithSpeed(Rotate,0.85, rotateSpeed);
                    Slide.setTargetPosition(initialHeight + 500);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    sleep(500);
                    rotateWithSpeed(Rotate,0.25, rotateSpeed);
                    releasedB = false;
                }
            } else if(!releasedB){
                releasedB = true;
            }

            if(gamepad1.y){
                if(releasedY) {
                    LF.setPower(0);
                    RF.setPower(0);
                    LB.setPower(0);
                    RB.setPower(0);
                    rotateWithSpeed(Rotate,0.85, rotateSpeed);
                    Slide.setTargetPosition(initialHeight + 1150);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    sleep(500);
                    rotateWithSpeed(Rotate,0.25, rotateSpeed);
                    releasedY = false;
                }
            } else if(!releasedY){
                releasedY = true;
            }

            if (gamepad1.dpad_up) {
                if (releasedDU){
                    Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Slide.setPower(0.5);
                    releasedDU = false;
                }
            } else if (!releasedDU){
                Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Slide.setPower(0);
                releasedDU = true;
            }
            if (gamepad1.dpad_down) {
                if (releasedDD){
                    Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Slide.setPower(-0.5);
                    releasedDD = false;
                }
            } else if (!releasedDD){
                Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Slide.setPower(0);
                releasedDD = true;
            }

            if (gamepad1.right_trigger == 1) {
                if (releasedRT){
                    if (toggleRT) {
                        //if (currentState == DriveMethod.poseState)
                        spinPower = 1;
                        //twoPhaseSpin(false, 0.7);
                        telemetry.addLine("SPIN STARTS");
                        toggleRT = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("SPIN STOPS");
                        toggleRT = true;
                    }
                    releasedRT = false;
                }
            } else if (!releasedRT){
                releasedRT = true;
            }

            if (gamepad1.left_trigger == 1) {
                if (releasedLT){
                    if (toggleLT) {
                        spinPower = -1;
                        //twoPhaseSpin(true, 0.7);
                        telemetry.addLine("SPIN STARTS REVERSE");
                        toggleLT = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("SPIN STOPS");
                        toggleLT = true;
                    }
                    releasedLT = false;
                }
            } else if (!releasedLT){
                releasedLT = true;
            }

            LFPower  = Range.clip(speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(speed*(drive - rotate - strafe), -1.0, 1.0) ;

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);
            Intake.setPower(intakePower);
            Spin.setPower(spinPower);

            telemetry.addLine("Rotate: " + Rotate.getPosition());
            telemetry.addLine("Arm: " + Arm.getPosition());
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

    private void rotateWithSpeed(Servo s, double targetPos, double factor){
        //1 -> 10 0.5 -> 5
        if(factor == 1.0){
            s.setPosition(targetPos);
        }
        double currentPos = s.getPosition();
        double interval = 0.05 * factor;
        while (targetPos > s.getPosition()){
            s.setPosition(s.getPosition() + interval);
            sleep(30);
        }
        while (targetPos < s.getPosition()){
            s.setPosition(s.getPosition() - interval);
            sleep(30);
        }
    }

}