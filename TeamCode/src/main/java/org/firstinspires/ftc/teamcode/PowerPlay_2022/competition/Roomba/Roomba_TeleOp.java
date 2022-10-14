package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Roomba TeleOp", group = "1")
public class Roomba_TeleOp extends LinearOpMode {
    private DcMotor LF, RF, LB, RB, Slide;
    private CRServo Turn;
    private Servo Pinch;

    private double speed = Roomba_Constants.INITIAL_SPEED;

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
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);

        // Use slide positions
        final int SLIDE_INITIAL = Slide.getCurrentPosition();
        int slideStoppedPos = SLIDE_INITIAL;

        // Set power variables
        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        // Initialize pinch position
        Pinch.setPosition(Roomba_Constants.PINCH_MIN);

        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Gamepad 1
        boolean releasedA1 = true, releasedB1 = true;
        boolean releasedDU1 = true, releasedDD1 = true;
        boolean releasedRB1 = true;
        boolean releasedLT1 = true;

        // Gamepad 2
        boolean releasedA2 = true, releasedB2 = true, releasedX2 = true, releasedY2 = true;
        boolean releasedDL2 = true, releasedDR2 = true, releasedDU2 = true, releasedDD2 = true;
        boolean releasedLB2 = true, releasedRB2 = true;
        boolean releasedLT2 = true, releasedRT2 = true;

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.a) {
                speed = Roomba_Constants.LOW_SPEED;
                releasedA1 = false;
            } else if (!releasedA1) {
                releasedA1 = true;
            }

            if (gamepad1.b) {
                speed = Roomba_Constants.HIGH_SPEED;
                releasedB1 = false;
            } else if (!releasedB1) {
                releasedB1 = true;
            }

            if (gamepad1.dpad_up) {
                if (releasedDU1) {
                    increaseSpeed(Roomba_Constants.SPEED_INCREMENT);
                    releasedDU1 = false;
                }
            } else if (!releasedDU1) {
                releasedDU1 = true;
            }

            if (gamepad1.dpad_down) {
                if (releasedDD1) {
                    decreaseSpeed(Roomba_Constants.SPEED_INCREMENT);
                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                releasedDD1 = true;
            }

            if (gamepad1.right_bumper) {
                if (releasedRB1) {
                    if (Pinch.getPosition() > Roomba_Constants.PINCH_MIDPOINT) {
                        Pinch.setPosition(Roomba_Constants.PINCH_MIN);
                    } else {
                        Pinch.setPosition(Roomba_Constants.PINCH_MAX);
                    }
                    releasedRB1 = false;
                }
            } else if (!releasedRB1) {
                releasedRB1 = true;
            }

            if (gamepad2.a) {
                if (releasedA2) {
                    Slide.setTargetPosition(SLIDE_INITIAL);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.7);
                    releasedA2 = false;
                }
            } else if (!releasedA2) {
                releasedA2 = true;
            }

            if (gamepad2.b) {
                if (releasedB2) {
                    Slide.setTargetPosition(SLIDE_INITIAL + Roomba_Constants.SL_LOW);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedB2 = false;
                }
            } else if (!releasedB2) {
                releasedB2 = true;
            }

            if (gamepad2.x) {
                if (releasedX2) {
                    Slide.setTargetPosition(SLIDE_INITIAL + Roomba_Constants.SL_MEDIUM);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedX2 = false;
                }
            } else if (!releasedX2) {
                releasedX2 = true;
            }

            if (gamepad2.y) {
                if (releasedY2) {
                    Slide.setTargetPosition(SLIDE_INITIAL + Roomba_Constants.SL_HIGH);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedY2 = false;
                }
            } else if (!releasedY2) {
                releasedY2 = true;
            }

            if (gamepad2.dpad_up) {
                if (releasedDU2) {
                    Slide.setPower(0.5);
                    releasedDU2 = false;
                }
            } else if (!releasedDU2){
                Slide.setPower(0);
                slideStoppedPos = Slide.getCurrentPosition();
                //Slide.setTargetPosition(slideStoppedPos);
                //Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                releasedDU2 = true;
            }

            if (gamepad2.dpad_down) {
                if (releasedDD2) {
                    Slide.setPower(-0.5);
                    releasedDD2 = false;
                }
            } else if (!releasedDD2) {
                Slide.setPower(0);
                slideStoppedPos = Slide.getCurrentPosition();
                //Slide.setTargetPosition(slideStoppedPos);
                //Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                releasedDD2 = true;
            }

            if (gamepad2.dpad_left) {
                if (releasedDL2) {
                    Turn.setPower(Roomba_Constants.TURN_MIN);
                    releasedDL2 = false;
                }
            } else if (!releasedDL2) {
                Turn.setPower(0);
                releasedDL2 = true;
            }

            if (gamepad2.dpad_right) {
                if (releasedDR2) {
                    Turn.setPower(Roomba_Constants.TURN_MAX);
                    releasedDR2 = false;
                }
            } else if (!releasedDR2) {
                Turn.setPower(0);
                releasedDR2 = true;
            }

            /*
            if (gamepad2.left_trigger > 0.9) { //&& Slide.getCurrentPosition() >= SLIDE_INITIAL + Roomba_Constants.SL_LOW
                if (releasedLT2) {
                    if (Turn.getPosition() > Roomba_Constants.TURN_MIDPOINT) Turn.setPosition(Roomba_Constants.TURN_MIN);
                    else Turn.setPosition(Roomba_Constants.TURN_MAX);
                    releasedLT2 = false;
                }
            } else if (!releasedLT2) {
                releasedLT2 = true;
            }
            */

            if (gamepad2.right_bumper) {
                if (releasedRB2) {
                    if (Pinch.getPosition() > Roomba_Constants.PINCH_MIDPOINT) {
                        Pinch.setPosition(Roomba_Constants.PINCH_MIN);
                    } else {
                        Pinch.setPosition(Roomba_Constants.PINCH_MAX);
                    }
                    releasedRB2 = false;
                }
            } else if (!releasedRB2) {
                releasedRB2 = true;
            }

            LFPower = Range.clip(speed * (drive + rotate - strafe), -1.0, 1.0);
            LBPower = Range.clip(speed * (drive + rotate + strafe), -1.0, 1.0);
            RFPower = Range.clip(speed * (drive - rotate + strafe), -1.0, 1.0);
            RBPower = Range.clip(speed * (drive - rotate - strafe), -1.0, 1.0);

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);

            /*
            if (Slide.getCurrentPosition() > SLIDE_INITIAL + 600) {
                Turn.setPower(Range.clip(gamepad2.right_stick_x, Roomba_Constants.TURN_MIN, Roomba_Constants.TURN_MAX));
            } else {
                Turn.setPower(0);
            }
            */

            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
            telemetry.addLine("Turn: " + Turn.getPower());
            telemetry.addLine("Pinch: " + Pinch.getPosition());
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