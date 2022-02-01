package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.Chassis2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.DriveMethod;
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.FieldConstant;
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive_Chassis2;

@TeleOp(name = "TELEOP FINAL 2", group = "A Competition")
public class Mecanum_TeleOp_Final extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Slide = null;
    private CRServo Spin = null;
    private Servo Rotate = null;
    private Servo Arm = null;
    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.6;
    boolean reverse = false;

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

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        double intakePower = 0;
        double spinPower = 0;
        int initialHeight = Slide.getCurrentPosition();

        //Read Position From Auto
        SampleMecanumDrive_Chassis2 chassis = new SampleMecanumDrive_Chassis2(hardwareMap);
        chassis.setPoseEstimate(new Pose2d(0, 0, chassis.getExternalHeading()));
        //chassis.setPoseEstimate(PoseStorage.currentPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Rotate to initial position
        rotateWithSpeed(Rotate,0.95, 1);

        waitForStart();

        boolean releasedRB1 = true;
        boolean releasedLB1 = true;
        boolean releasedX1 = true;
        boolean releasedRB2 = true;
        boolean releasedLB2 = true;
        boolean releasedLT2 = true;
        boolean releasedRT2 = true;
        boolean releasedA1 = true;
        boolean releasedA2 = true;
        boolean releasedB1 = true;
        boolean releasedB2 = true;
        boolean releasedY1 = true;
        boolean releasedY2 = true;
        boolean releasedX2 = true;
        boolean releasedDD1 = true;
        boolean releasedDD2 = true;
        boolean releasedDL2 = true;
        boolean releasedDU1 = true;
        boolean releasedDU2 = true;
        boolean releasedDR2 = true;
        boolean releasedBX1 = true;
        boolean releasedBB1 = true;

        boolean releasedBack2 =true;
        boolean releasedStart2 =true;
        double rotateSpeed = 1;

        boolean toggleX1 = true;
        boolean toggleRB2 = true;
        boolean toggleLB2 = true;
        boolean toggleLT2 = true;

        while (opModeIsActive()) {
            runtime.reset();
            chassis.update();

            // Retrieve your pose
            Pose2d myPose = chassis.getPoseEstimate();
            DriveMethod.poseState currentState = DriveMethod.poseToState(myPose);
            telemetry.addLine(currentState.toString());

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            //////////////GAMEPAD 1//////////////
            if(gamepad1.back && gamepad1.x && currentState == DriveMethod.poseState.UNKNOWN){
                if(releasedBX1) {
                    currentState = DriveMethod.poseState.BLUE;
                    releasedBX1 = false;
                }
            } else if(!releasedBX1){
                releasedBX1 = true;
            }

            if(gamepad1.back && gamepad1.b && currentState == DriveMethod.poseState.UNKNOWN){
                if(releasedBB1) {
                    currentState = DriveMethod.poseState.RED;
                    releasedBB1 = false;
                }
            } else if(!releasedBB1){
                releasedBB1 = true;
            }

            if(gamepad1.dpad_up) {
                if(releasedDU1) {
                    increaseSpeed(0.05);
                    releasedDU1 = false;
                }
            } else if(!releasedDU1){
                releasedDU1 = true;
            }

            if(gamepad1.dpad_down){
                if(releasedDD1) {
                    decreaseSpeed(0.05);
                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                releasedDD1 = true;
            }
            if (gamepad1.left_bumper) {
                if (releasedLB1 && Slide.getCurrentPosition() < initialHeight + 30){
                    intakePower = 0.6;
                    telemetry.addLine("INTAKE STARTS");
                    releasedLB1 = false;
                }
            } else if (!releasedLB1){
                intakePower = 0;
                telemetry.addLine("INTAKE STOPS");
                releasedLB1 = true;
            }
            if (gamepad1.right_bumper) {
                if (releasedRB1 && Slide.getCurrentPosition() < initialHeight + 30){
                    intakePower = -0.6;
                    telemetry.addLine("INTAKE REVERSE STARTS");
                    releasedRB1 = false;
                }
            } else if (!releasedRB1){
                telemetry.addLine("INTAKE STOPS");
                intakePower = 0;
                releasedRB1 = true;
            }

            if(gamepad1.a && !gamepad1.start){
                if(releasedA1) {
                    speed = 0.25;
                    releasedA1 = false;
                }
            } else if(!releasedA1){
                releasedA1 = true;
            }

            if(gamepad1.b && !gamepad1.start){
                if(releasedB1) {
                    speed = 0.6;
                    releasedB1 = false;
                }
            } else if(!releasedB1){
                releasedB1 = true;
            }


            if (gamepad1.x) {
                if (releasedX1){
                    if (toggleX1) {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        telemetry.addLine("BREAK");
                        toggleX1 = false;
                    } else {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        telemetry.addLine("FLOAT");
                        toggleX1 = true;
                    }
                    releasedX1 = false;
                }
            } else if (!releasedX1){
                releasedX1 = true;
            }

            //SHARED HUB
            if(gamepad1.y) {
                if (releasedY1) {
                    if (PoseStorage.autoState == DriveMethod.poseState.BLUE) {
                        chassis.setPoseEstimate(FieldConstant.SHARED_BLUE_ENTER_POSE);
                        Trajectory sharedTraj1 = chassis.trajectoryBuilder(chassis.getPoseEstimate(), true)
                                .lineTo(new Vector2d(64.75, 18))
                                .build();
                        chassis.followTrajectory(sharedTraj1);
                        Trajectory sharedTraj2 = chassis.trajectoryBuilder(sharedTraj1.end(), true)
                                .lineToLinearHeading(FieldConstant.SHARED_BLUE_END_POSE)
                                .build();
                        chassis.followTrajectory(sharedTraj2);
                    } else if (PoseStorage.autoState == DriveMethod.poseState.RED) {
                        chassis.setPoseEstimate(FieldConstant.SHARED_RED_ENTER_POSE);
                        Trajectory sharedTraj1 = chassis.trajectoryBuilder(chassis.getPoseEstimate(), true)
                                .lineTo(new Vector2d(64.75, -18))
                                .build();
                        chassis.followTrajectory(sharedTraj1);
                        Trajectory sharedTraj2 = chassis.trajectoryBuilder(sharedTraj1.end(), true)
                                .lineToLinearHeading(FieldConstant.SHARED_RED_END_POSE)
                                .build();
                        chassis.followTrajectory(sharedTraj2);
                    } else {
                        gamepad1.rumble(300);
                    }
                    speed = 0.3;
                    chassis.updatePoseEstimate();
                    PoseStorage.state = DriveMethod.poseToState(chassis.getPoseEstimate());
                    releasedY1 = false;
                } else if (!releasedY1) {
                    speed = 0.2;
                    releasedY1 = true;
                }
            }

            //////////////GAMEPAD 2//////////////
            if(gamepad2.a && !gamepad2.start){
                if(releasedA2) {
                    LF.setPower(0);
                    RF.setPower(0);
                    LB.setPower(0);
                    RB.setPower(0);
                    rotateWithSpeed(Rotate,0.95, rotateSpeed);
                    Slide.setTargetPosition(initialHeight);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    releasedA2 = false;
                }

            } else if(!releasedA2){
                releasedA2 = true;
            }
            if(gamepad2.b && !gamepad2.start){
                if(releasedB2) {
                    LF.setPower(0);
                    RF.setPower(0);
                    LB.setPower(0);
                    RB.setPower(0);
                    Slide.setTargetPosition(initialHeight + 500);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    sleep(500);
                    rotateWithSpeed(Rotate,0.25, rotateSpeed);
                    releasedB2 = false;
                }
            } else if(!releasedB2){
                releasedB2 = true;
            }

            if(gamepad2.y){
                if(releasedY2) {
                    LF.setPower(0);
                    RF.setPower(0);
                    LB.setPower(0);
                    RB.setPower(0);
                    Slide.setTargetPosition(initialHeight + 1150);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    sleep(500);
                    rotateWithSpeed(Rotate,0.25, rotateSpeed);
                    releasedY2 = false;
                }
            } else if(!releasedY2){
                releasedY2 = true;
            }

            if(gamepad2.x){
                if(releasedX2) {
                    rotateWithSpeed(Rotate,0.90, rotateSpeed);
                    sleep(500);
                    Slide.setTargetPosition(initialHeight);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    Rotate.setPosition(0.95);
                    releasedX2 = false;
                }
            } else if(!releasedX2){
                releasedX2 = true;
            }
            if (gamepad2.dpad_up) {
                if (releasedDU2){
                    Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Slide.setPower(0.5);
                    releasedDU2 = false;
                }
            } else if (!releasedDU2){
                Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Slide.setPower(0);
                releasedDU2 = true;
            }
            if (gamepad2.dpad_down) {
                if (releasedDD2){
                    Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Slide.setPower(-0.5);
                    releasedDD2 = false;
                }
            } else if (!releasedDD2){
                Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Slide.setPower(0);
                releasedDD2 = true;
            }
            if (gamepad2.dpad_left) {
                if (releasedDL2){
                    if(Rotate.getPosition() <= 0.95 && gamepad2.dpad_left) {
                        Rotate.setPosition(Rotate.getPosition()+0.05);
                    }
                    releasedDL2 = false;
                }
            } else if (!releasedDL2){
                releasedDL2 = true;
            }
            if (gamepad2.dpad_right) {
                if (releasedDR2){
                    if(Rotate.getPosition() >= 0.25 && gamepad2.dpad_right) {
                        Rotate.setPosition(Rotate.getPosition()-0.05);
                    }
                    releasedDR2 = false;
                }
            } else if (!releasedDR2){
                releasedDR2 = true;
            }

            if(gamepad2.back){
                if(releasedBack2){
                    rotateSpeed = 0.5;
                    releasedBack2 = false;
                }else if (!releasedBack2){
                    releasedBack2 = true;
                }
            }
            if(gamepad2.start){
                if(releasedStart2){
                    rotateSpeed = 1.0;
                    releasedStart2 = false;
                }else if (!releasedStart2){
                    releasedStart2 = true;
                }
            }

            if (gamepad2.right_bumper) {
                if (releasedRB2){
                    if (toggleRB2) {
                        //if (currentState == DriveMethod.poseState)
                        spinPower = 1;
                        //twoPhaseSpin(false, 0.7);
                        telemetry.addLine("SPIN STARTS");
                        toggleRB2 = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("SPIN STOPS");
                        toggleRB2 = true;
                    }
                    releasedRB2 = false;
                }
            } else if (!releasedRB2){
                releasedRB2 = true;
            }

            if (gamepad2.left_bumper) {
                if (releasedLB2){
                    if (toggleLB2) {
                        spinPower = -1;
                        //twoPhaseSpin(true, 0.7);
                        telemetry.addLine("SPIN STARTS REVERSE");
                        toggleLB2 = false;
                    } else {
                        spinPower = 0;
                        telemetry.addLine("SPIN STOPS");
                        toggleLB2 = true;
                    }
                    releasedLB2 = false;
                }
            } else if (!releasedLB2){
                releasedLB2 = true;
            }


            if(gamepad2.left_trigger == 1){
                if (releasedLT2){
                    if (toggleLT2 && Rotate.getPosition() <= 0.5) {
                        rotateWithSpeed(Rotate, 0.95, rotateSpeed);
                        telemetry.addLine("ROTATE STARTS");
                        toggleLT2 = false;
                    } else {
                        rotateWithSpeed(Rotate, 0.25, rotateSpeed);
                        telemetry.addLine("ROTATE STOPS");
                        toggleLT2 = true;
                    }
                    releasedLT2 = false;
                }
            } else if (!releasedLT2){
                releasedLT2 = true;
            }

            if(gamepad2.right_trigger == 1){
                if (releasedRT2){
                    Arm.setPosition(0);
                    releasedRT2 = false;
                }
            } else if (!releasedRT2){
                Arm.setPosition(0.5);
                releasedRT2 = true;
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
            telemetry.addLine();
            telemetry.addData("LF: ", LF.getCurrentPosition());
            telemetry.addData("RF: ", RF.getCurrentPosition());
            telemetry.addData("LB: ", LB.getCurrentPosition());
            telemetry.addData("RB: ", RB.getCurrentPosition());

            telemetry.update();

        }
    }

    void twoPhaseSpin(boolean isReversed,double startingSpeed) {
        double reverseFactor = 1;
        if(isReversed){
            reverseFactor = -1;
        }
        ElapsedTime tSpin = new ElapsedTime();
        double spinPower = startingSpeed * reverseFactor;
        while (tSpin.milliseconds() < 800){
            Spin.setPower(spinPower);
        }
        while (tSpin.milliseconds() < 1500){
            spinPower = Range.clip(spinPower * tSpin.milliseconds()/800.0, -1.0, 1.0);
            Spin.setPower(spinPower);
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