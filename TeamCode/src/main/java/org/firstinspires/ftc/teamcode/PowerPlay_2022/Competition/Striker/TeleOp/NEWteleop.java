package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Striker.TeleOp;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;

@Disabled
@TeleOp(name = "NewTeleop", group = "Competition")
public class NEWteleop extends LinearOpMode {
    private DcMotor LF, RF, LB, RB, Slide, Turn;
    private Servo Pinch;

    private double speed = RoombaConstants.INITIAL_SPEED;

    @Override
    public void runOpMode() {
        // Get devices from hardware map
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Turn = hardwareMap.get(DcMotor.class, "Turn");
        Pinch = hardwareMap.get(Servo.class, "Pinch");

        // Initialize devices
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        //Turn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Turn.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use slide positions
        int slideInitial = Slide.getCurrentPosition();
        Slide.setTargetPosition(slideInitial);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int turninitial = Turn.getCurrentPosition();
        Turn.setTargetPosition(turninitial);
        Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set power variables
        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Gamepad 1
        boolean releasedA1 = true, releasedB1 = true, releasedY1 = true;
        boolean releasedDU1 = true, releasedDD1 = true;
        boolean releasedRB1 = true;
        boolean releasedLT1 = true;
        boolean releasedDR1 = true;
        boolean releasedDL1 = true;
        boolean releasedLB1 = true;

        // Gamepad 2
        boolean releasedA2 = true, releasedB2 = true, releasedX2 = true, releasedY2 = true;
        boolean releasedDL2 = true, releasedDR2 = true, releasedDU2 = true, releasedDD2 = true;
        boolean releasedLB2 = true, releasedRB2 = true;
        boolean releasedLT2 = true, releasedRT2 = true;

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;


//gamepad2

            if (gamepad1.a) {
                if (releasedA2) {
                    Slide.setTargetPosition(slideInitial + 5);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    releasedA2 = false;
                }
            } else if (!releasedA2) {
                releasedA2 = true;
            }
//gamepad2
            if (gamepad1.b) {
                if (releasedB2) {
                    Slide.setTargetPosition(slideInitial + 1505);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    releasedB2 = false;
                }
            } else if (!releasedB2) {
                releasedB2 = true;
            }
//gamepad2
            if (gamepad1.x) {
                if (releasedX2) {
                    Slide.setTargetPosition(slideInitial + 2600);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    releasedX2 = false;
                }
            } else if (!releasedX2) {
                releasedX2 = true;
            }
//gamepad2
            if (gamepad1.y) {
                if (releasedY2) {
                    Slide.setTargetPosition(slideInitial + 5700);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    releasedY2 = false;
                }
            } else if (!releasedY2) {
                releasedY2 = true;
            }
//gamepad2
            if (gamepad1.dpad_up) {
                if (releasedDU2) {
                    Slide.setTargetPosition(Slide.getCurrentPosition() + 150);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    releasedDU2 = false;
                }
            } else if (!releasedDU2){
                releasedDU2 = true;
            }
//gamepad2
            if (gamepad1.dpad_down) {
                if (releasedDD2) {
                    Slide.setTargetPosition(Slide.getCurrentPosition() - 150);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(1);
                    releasedDD2 = false;
                }
            } else if (!releasedDD2) {
                releasedDD2 = true;
            }

            
            if (gamepad1.right_bumper) {
                Pinch.setPosition(0.67);
            } else {
                Pinch.setPosition(1);
            }


            if (gamepad1.dpad_right) {
                if (releasedDR1) {
                    speed = 0.35;
                }
            }


            if (gamepad1.dpad_left) {
                if (releasedDL1) {
                    speed = 0.65;
                }
            }


            if (gamepad1.right_trigger >= 0.7) {
                Turn.setTargetPosition(1000);
                //Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Turn.setPower(0.6);
            } else {
                Turn.setPower(0);
            }

            if (gamepad1.left_trigger >= 0.7) {
                Turn.setTargetPosition(-1000);
                //Turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Turn.setPower(0.6);
            } else {
                Turn.setPower(0);
            }




            LFPower = Range.clip(speed * (drive + rotate - strafe), -1.0, 1.0);
            LBPower = Range.clip(speed * (drive + rotate + strafe), -1.0, 1.0);
            RFPower = Range.clip(speed * (drive - rotate + strafe), -1.0, 1.0);
            RBPower = Range.clip(speed * (drive - rotate - strafe), -1.0, 1.0);

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);




            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
            telemetry.addLine("Turn: " + Turn.getPower());
            telemetry.addLine("TurnPosition: " + Turn.getCurrentPosition());
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
