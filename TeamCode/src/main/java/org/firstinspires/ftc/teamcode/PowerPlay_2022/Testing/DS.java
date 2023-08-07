package org.firstinspires.ftc.teamcode.PowerPlay_2022.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DS Tester", group = "Testing")
public class DS extends LinearOpMode {
    private ColorSensor Color;
    private DistanceSensor Distance;

    @Override
    public void runOpMode() {
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        Color = hardwareMap.get(ColorSensor.class, "Color");

        Color.enableLed(true);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance", String.format("%.01f mm", Distance.getDistance(DistanceUnit.MM)));
            telemetry.addData("RED", Color.red());
            telemetry.addData("BLUE", Color.blue());
            telemetry.addData("GREEN", Color.green());
            telemetry.update();
        }
    }
}
