package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum TeleOp", group = "1")

public class Mecanum_TeleOp extends LinearOpMode{
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    double speed = 1;

    @Override
    public void runOpMode() {
        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean releasedDD1 = true;
        boolean releasedDU1 = true;

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

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

            LFPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(gamepad1.left_trigger + speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(gamepad1.right_trigger + speed*(drive - rotate - strafe), -1.0, 1.0) ;

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);

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
