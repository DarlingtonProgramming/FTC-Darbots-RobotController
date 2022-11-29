package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;

@TeleOp(name = "Roomba TeleOp Old", group = "Competition")
public class RoombaTeleOp extends LinearOpMode {
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
        RB.setDirection(DcMotor.Direction.FORWARD);
        Pinch.setDirection(Servo.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use slide positions
        int slideInitial = Slide.getCurrentPosition();
        Slide.setTargetPosition(slideInitial);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power variables
        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;
        double TNPower;


        double turnSpeed = 0.5;


        // Update telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Gamepad 1
        boolean releasedA1 = true, releasedB1 = true, releasedY1 = true;
        boolean releasedDU1 = true, releasedDD1 = true;
        boolean releasedRB1 = true;
        boolean releasedLT1 = true;

        // Gamepad 2
        boolean releasedA2 = true, releasedB2 = true, releasedX2 = true, releasedY2 = true;
        boolean releasedDL2 = true, releasedDR2 = true, releasedDU2 = true, releasedDD2 = true;
        boolean releasedLB1 = true, releasedLB2 = true, releasedRB2 = true;
        boolean releasedLT2 = true, releasedRT2 = true;

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            double turret = gamepad2.left_stick_x;


            //Lowers Speed
            if (gamepad1.a) {
                speed = RoombaConstants.LOW_SPEED;
                releasedA1 = false;
            } else if (!releasedA1) {
                releasedA1 = true;
            }

            //Increase Speed
            if (gamepad1.b) {
                speed = RoombaConstants.HIGH_SPEED;
                releasedB1 = false;
            } else if (!releasedB1) {
                releasedB1 = true;
            }

            //Increase Wheels Speed
            if (gamepad1.dpad_up) {
                if (releasedDU1) {
                    increaseSpeed(RoombaConstants.SPEED_INCREMENT);
                    releasedDU1 = false;
                }
            } else if (!releasedDU1) {
                releasedDU1 = true;
            }

            //Decrease Wheels Speed
            if (gamepad1.dpad_down) {
                if (releasedDD1) {
                    decreaseSpeed(RoombaConstants.SPEED_INCREMENT);
                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                releasedDD1 = true;
            }
//Push this to git 
            //Turn speed to left
            if (gamepad1.right_bumper) {
                if (releasedRB1) {
                    Pinch.setPosition(.9);
                    releasedRB1 = false;
                }
            } else if (!releasedRB1) {
                releasedRB1 = true;
            }

            if (gamepad1.left_bumper) {
                if (releasedLB1) {
                    Pinch.setPosition(.5);
                    releasedLB1 = false;
                }
            } else if (!releasedLB1) {
                releasedLB1 = true;
            }

            // Go to Ground
            if (gamepad2.a) {
                if (releasedA2) {
                    Slide.setTargetPosition(slideInitial);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.7);
                    releasedA2 = false;
                }
            } else if (!releasedA2) {
                releasedA2 = true;
            }

            // Go to Low Junction
            if (gamepad2.b) {
                if (releasedB2) {
                    Slide.setTargetPosition(slideInitial + RoombaConstants.SL_LOW);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(-0.7);
                    releasedB2 = false;
                }
            } else if (!releasedB2) {
                releasedB2 = true;
            }

            //Go to Medium Junciton
            if (gamepad2.x) {
                if (releasedX2) {
                    Slide.setTargetPosition(slideInitial + RoombaConstants.SL_MEDIUM);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedX2 = false;
                }
            } else if (!releasedX2) {
                releasedX2 = true;
            }

            //Go to High Junction
            if (gamepad2.y) {
                if (releasedY2) {
                    Slide.setTargetPosition(slideInitial + RoombaConstants.SL_HIGH);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedY2 = false;
                }
            } else if (!releasedY2) {
                releasedY2 = true;
            }

            //Increase slide slightly
            if (gamepad2.dpad_up) {
                if (releasedDU2) {
                    Slide.setTargetPosition(Slide.getCurrentPosition() + 105);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedDU2 = false;
                }
            } else if (!releasedDU2){
                releasedDU2 = true;
            }

            //Decrease slide slightly
            if (gamepad2.dpad_down) {
                if (releasedDD2) {
                    Slide.setTargetPosition(Slide.getCurrentPosition() - 105);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(0.8);
                    releasedDD2 = false;
                }
            } else if (!releasedDD2) {
                releasedDD2 = true;
            }

            //Reset Slide position.
            if (gamepad2.left_bumper) {
                if (releasedLB2) {
                    slideInitial = Slide.getCurrentPosition();
                    releasedLB2 = false;
                }
            } else if (!releasedLB2) {
                releasedLB2 = true;
            }

            LFPower = Range.clip(speed * (drive + rotate + strafe), -1.0, 1.0);
            LBPower = Range.clip(speed * (drive + rotate - strafe), -1.0, 1.0);
            RFPower = Range.clip(speed * (drive - rotate - strafe), -1.0, 1.0);
            RBPower = Range.clip(speed * (drive - rotate + strafe), -1.0, 1.0);

            TNPower = Range.clip((turnSpeed * (turret)), -1.0,1.0);

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);

            Turn.setPower(TNPower);

            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
            telemetry.addLine("Pinch: " + Pinch.getPosition());
            telemetry.addLine("Turn: " + (turret));
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("Wheels Speed:", speed);
            telemetry.addData("Turret Speed:", turnSpeed);
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

    /*private void pinch(){

    }
    */
    private void increaseSpeed(double s) {
        double increased = speed + s;
        if (1 < increased) {
            speed = 1;
            return;
        }
        speed = increased;
    }
}


