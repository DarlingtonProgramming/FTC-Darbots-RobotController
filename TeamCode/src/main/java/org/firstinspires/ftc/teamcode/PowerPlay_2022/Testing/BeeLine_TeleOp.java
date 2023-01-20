package org.firstinspires.ftc.teamcode.PowerPlay_2022.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "Beeline TeleOp", group = "1")

public class BeeLine_TeleOp extends LinearOpMode {
    private DcMotor leftMotors = null;
    private DcMotor rightMotors = null;

    double speed = 1;

    @Override
    public void runOpMode() {
        leftMotors = hardwareMap.get(DcMotor.class, "leftMotors");
        rightMotors = hardwareMap.get(DcMotor.class, "rightMotors");

        leftMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean releasedDD1 = true;
        boolean releasedDU1 = true;

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.dpad_up) {
                if (releasedDU1) {
                    increaseSpeed(0.05);
                    releasedDU1 = false;
                }
            } else if (!releasedDU1) {
                releasedDU1 = true;
            }

            if (gamepad1.dpad_down) {
                if (releasedDD1) {
                    decreaseSpeed(0.05);
                    releasedDD1 = false;
                }
            } else if (!releasedDD1) {
                releasedDD1 = true;
            }

            leftMotors.setPower(Range.clip(gamepad1.left_trigger + speed * (drive + rotate), -1.0, 1.0));
            rightMotors.setPower(Range.clip(gamepad1.left_trigger + speed * (drive - rotate), -1.0, 1.0));

            telemetry.addData("Left Motors", "(%.2f)", leftMotors.getPower());
            telemetry.addData("Right Motors", "(%.2f)", rightMotors.getPower());
            telemetry.addData("Controller", "-Y (%.2f), X (%.2f)", drive, rotate);
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
