package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.mason;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.DriveMethod;

import static java.lang.Thread.sleep;

@Autonomous(name = "Drawbot Auto Line", group = "Drawbot")
@Disabled
public class Drawbot_AutoLine extends LinearOpMode {
    private DcMotor L = null;
    private DcMotor R = null;
    private Servo LArm = null;
    private Servo RArm = null;

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        L  = hardwareMap.get(DcMotor.class, "L");
        R = hardwareMap.get(DcMotor.class, "R");

        L.setDirection(DcMotor.Direction.FORWARD);
        R.setDirection(DcMotor.Direction.REVERSE);

        LArm  = hardwareMap.get(Servo.class, "LArm");
        RArm = hardwareMap.get(Servo.class, "RArm");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double LPower = 0;
        double RPower = 0;
        double speed = 0.8;

        LArm.setPosition(0);
        RArm.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            drive(0.5, 0.5, 500);
            rotateToAngle(90.0, 4.0, 0.5);
            drive(0.5, 0.5, 500);
            rotateToAngle(0.0, 4.0, 0.5);
            drive(0.5, 0.5, 500);
            drive(0, 0, 100);
        }
    }

    void drive(double LPower, double RPower, long duration) {
        L.setPower(LPower);
        R.setPower(RPower);
        sleep(duration);
    }

    void stopMotion() {
        L.setPower(0);
        R.setPower(0);

    }

    double normalizeAngle(double angle) {
        double tempDeg = angle % 360;
        if (tempDeg >= 180) {
            tempDeg -= 360;
        } else if (tempDeg < -180) {
            tempDeg += 360;
        }
        return tempDeg;
    }

    double aquireHeading() throws InterruptedException {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempHead = normalizeAngle(heading);
        sleep(20);
        return tempHead;
    }

    void rotateAtAngle(boolean isClockwise, double degree, double margin, double power) throws InterruptedException {
        int angleFactor = -1;
        if (!isClockwise) {
            angleFactor = 1;
        }
        final double currentAngle = aquireHeading();
        double targetAngle = normalizeAngle(currentAngle + degree * angleFactor);
        rotateToAngle(targetAngle, margin, power);
        stopMotion();
    }

    //make a turn TO a certain angle
    void rotateToAngle(double targetAngle, double margin, double power) throws InterruptedException {
        int angleFactor = 0;
        final double currentAngle = aquireHeading();
        if (currentAngle - targetAngle > 0) {
            if (currentAngle - targetAngle < 180) {
                //cw
                angleFactor = -1;
            } else {
                //ccw
                angleFactor = 1;
            }
        } else {
            if (targetAngle - currentAngle < 180) {
                //ccw
                angleFactor = 1;
            } else {
                //cw
                angleFactor = -1;
            }
        }
        double R_power;
        double L_power;

        double tempAngle = currentAngle;
        while (!((tempAngle < targetAngle + margin) && (tempAngle > targetAngle - margin))) {
            tempAngle = aquireHeading();
            R_power = angleFactor * power;
            L_power = -1 * angleFactor * power;
            R_power = Range.clip(R_power, -1, 1);
            L_power = Range.clip(L_power, -1, 1);
            L.setPower(L_power);
            R.setPower(R_power);
//            telemetry.addData("RF_power", RF_power);
//            telemetry.addData("RB_power", RB_power);
//            telemetry.addData("LF_power", LF_power);
//            telemetry.addData("LB_power", LB_power);
//            telemetry.update();
        }
        stopMotion();
    }
}

