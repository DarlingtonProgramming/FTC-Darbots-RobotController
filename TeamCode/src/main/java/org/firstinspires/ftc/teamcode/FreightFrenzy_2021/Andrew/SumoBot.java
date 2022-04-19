package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.Andrew;

import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="SumoBOT OpMode", group="Linear Opmode")

public class SumoBot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    double speed = 0.6;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean releasedDD1 = true;
        boolean releasedDU1 = true;
        boolean releasedX = true;
        boolean switchMode = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            runtime.reset();

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


            if (gamepad1.x) {
                if (releasedX) {
                    if (switchMode){
                        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        telemetry.addLine("BREAK");
                        switchMode=false;
                    } else if (!switchMode) {
                        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        telemetry.addLine("FLOAT");
                        switchMode = true;
                    }
                    releasedX = false;
                }

            } else if(!releasedX){
                releasedX=true;
            }


                // Setup a variable for each drive wheel to save power level for telemetry
                double leftPower;
                double rightPower;

                // Choose to drive using either Tank Mode, or POV Mode
                // Comment out the method that's not used.  The default below is POV.

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);

                // Tank Mode uses one stick to control each wheel.
                // - This requires no math, but it is hard to drive forward slowly and keep straight.
                // leftPower  = -gamepad1.left_stick_y ;
                // rightPower = -gamepad1.right_stick_y ;

                // Send calculated power to wheels
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
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
