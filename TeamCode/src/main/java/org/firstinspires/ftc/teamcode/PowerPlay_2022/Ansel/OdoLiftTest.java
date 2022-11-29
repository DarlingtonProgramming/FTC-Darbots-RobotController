package org.firstinspires.ftc.teamcode.PowerPlay_2022.Ansel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "OdoLiftTest", group = "test")
public class OdoLiftTest extends LinearOpMode {
    private DcMotor LF, RF, LB, RB, Slide;
    private Servo Pinch;
    private Servo OdoLift;

    private double speed = 0.5;

    @Override
    public void runOpMode() {
        // Get devices from hardware map
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        OdoLift = hardwareMap.get(Servo.class, "OdoLift");
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

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);

        // Use slide positions
        final int SLIDE_INITIAL = Slide.getCurrentPosition();

        // Set power variables
        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        // Initialize pinch position
        Pinch.setPosition(0.03);

        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Gamepad 1
        boolean releasedA1 = true, releasedB1 = true, releasedY1 = true, releasedX1 = true;
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
                speed = 0.35;
                releasedA1 = false;
            } else if (!releasedA1) {
                releasedA1 = true;
            }

            if (gamepad1.b) {
                speed = 0.6;
                releasedB1 = false;
            } else if (!releasedB1) {
                releasedB1 = true;
            }

            if (gamepad1.x) {
                OdoLift.setPosition(1);
                releasedX1 = false;
            } else if (!releasedX1) {
                releasedX1 = true;
            }

            if (gamepad1.y) {
                OdoLift.setPosition(0.56);
                releasedY1 = false;
            } else if (!releasedY1) {
                releasedY1 = true;
            }

            if (gamepad1.dpad_up) {
                if (releasedDU1) {
                    increaseSpeed(0.1);
                    releasedDU1 = false;
                }
            } else if (!releasedDU1) {
                releasedDU1 = true;
            }

            if (gamepad1.dpad_down) {
                if (releasedDD1) {
                    decreaseSpeed(-0.1);
                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                releasedDD1 = true;
            }

            if (gamepad1.right_bumper) {
                if (releasedRB1) {
                    if (Pinch.getPosition() > 0.5) {
                        Pinch.setPosition(0.03);
                    } else {
                        Pinch.setPosition(0.8);
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
                    Slide.setTargetPosition(SLIDE_INITIAL + 1200);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedB2 = false;
                }
            } else if (!releasedB2) {
                releasedB2 = true;
            }

            if (gamepad2.x) {
                if (releasedX2) {
                    Slide.setTargetPosition(SLIDE_INITIAL + 2000);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedX2 = false;
                }
            } else if (!releasedX2) {
                releasedX2 = true;
            }

            if (gamepad2.y) {
                if (releasedY2) {
                    Slide.setTargetPosition(SLIDE_INITIAL + 2800);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedY2 = false;
                }
            } else if (!releasedY2) {
                releasedY2 = true;
            }

            if (gamepad2.dpad_up) {
                if (releasedDU2) {
                    Slide.setTargetPosition(Slide.getCurrentPosition() + 105);
                    releasedDU2 = false;
                }
            } else if (!releasedDU2){
                releasedDU2 = true;
            }

            if (gamepad2.dpad_down) {
                if (releasedDD2) {
                    Slide.setTargetPosition(Slide.getCurrentPosition() - 105);
                    releasedDD2 = false;
                }
            } else if (!releasedDD2) {
                releasedDD2 = true;
            }

            /*
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
                    if (Pinch.getPosition() > 0.5) {
                        Pinch.setPosition(0.03);
                    } else {
                        Pinch.setPosition(0.8);
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
            telemetry.addLine("Pinch: " + Pinch.getPosition());
            telemetry.addLine("OdoLift: " + OdoLift.getPosition());
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