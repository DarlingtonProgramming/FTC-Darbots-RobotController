package org.firstinspires.ftc.teamcode.PowerPlay_2022.ansel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class Components {
    public Map<String, DcMotor> Motors = new HashMap<>();
    public Components(String[][] motorNames, HardwareMap hwMap) {
        setMotors(motorNames, hwMap);
    }
    private void setMotors(String[][] MotorNames, HardwareMap hwMap) {
        for (int i = MotorNames.length; i > 0; i--) {
            String[] data = MotorNames[i-1];
            Motors.put(data[0], hwMap.get(DcMotor.class, data[0]));

            DcMotor Motor = Motors.get(data[0]);
            Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Motor.setDirection(data[1].equals("FORWARD") ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        }
    }
}
