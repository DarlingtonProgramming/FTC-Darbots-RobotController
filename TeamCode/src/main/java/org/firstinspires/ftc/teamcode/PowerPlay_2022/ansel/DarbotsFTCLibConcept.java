package org.firstinspires.ftc.teamcode.PowerPlay_2022.ansel;

/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.darbots.darbotsftclib.libcore.templates.DarbotsAction;

@TeleOp(name = "Elysium-TeleOp", group = "4100")
public class DarbotsFTCLibConcept extends DarbotsBasicOpMode<a> {
    private ElysiumCore m_Core;
    private int telemetry_i = 0;
    public RobotMotionSystemTeleOpTask teleOpTask;

    @Override
    public ElysiumCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {

    }

    @Override
    public void hardwareDestroy() {
    }

    @Override
    public void RunThisOpMode() {
        this.teleopAutoMovement();
        while(this.opModeIsActive()){
            this.updateStatus();

            controlLoop();

            lazyTelemetry();
        }
    }

    public void teleopAutoMovement(){

    }

    public void controlLoop(){
        {

        }
        {

        }
    }
    public TelemetryPacket updateTelemetry(){
        TelemetryPacket packet = this.m_Core.updateTelemetry();
        return packet;
    }

    public void telemetryCycle(){
        TelemetryPacket packet = this.updateTelemetry();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }

    public void lazyTelemetry(){
        if(telemetry_i>=CONST_TELMETRY_PACKET_CYCLE_TIME) {
            telemetryCycle();
            telemetry_i = 0;
        }else{
            telemetry_i++;
        }
    }

    public void updateStatus(){
        this.m_Core.updateStatus();
        this.outtakeControlAction.updateStatus();
        this.intakePositioningAction.updateStatus();
    }
}

 */