package org.firstinspires.ftc.teamcode.PowerPlay_2022.Ansel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slide Test", group = "2")
public class SlideTest extends LinearOpMode {

    private DcMotor Slide = null;

    @Override
    public void runOpMode() {
        Slide = hardwareMap.get(DcMotor.class, "Slide");

        Slide.setDirection(DcMotor.Direction.FORWARD);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean releasedDD = true;
        boolean releasedDU = true;

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                if (releasedDU){
                    Slide.setPower(1);
                    releasedDU = false;
                }
            } else if (!releasedDU){
                Slide.setPower(0);
                releasedDU = true;
            }
            if (gamepad1.dpad_down) {
                if (releasedDD){
                    Slide.setPower(-1);
                    releasedDD = false;
                }
            } else if (!releasedDD){
                Slide.setPower(0);
                releasedDD = true;
            }

            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.addLine("Slide Target: " + Slide.getTargetPosition());

            telemetry.update();
        }
    }
}