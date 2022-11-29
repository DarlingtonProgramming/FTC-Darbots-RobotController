package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.PoseStorage;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.RecordingPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.RoombaDriveMethod;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.roadrunner.drive.MecanumDrive_Roomba;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Roomba TeleOp RR", group = "Competition")
public class RoombaTeleOpRR extends LinearOpMode {
    private DcMotor Slide;
    private Servo Pinch;
    private OpenCvWebcam Camera;
    private WebcamName Webcam;

    private double speed = RoombaConstants.INITIAL_SPEED;

    @Override
    public void runOpMode() {
        // Initialize roadrunner
        MecanumDrive_Roomba chassis = new MecanumDrive_Roomba(hardwareMap);
        chassis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chassis.setPoseEstimate(PoseStorage.currentPose);

        // Initialize others
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Pinch = hardwareMap.get(Servo.class, "Pinch");
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pinch.setDirection(Servo.Direction.REVERSE);

        // Initialize DriveMethod
        RoombaDriveMethod driveMethod = new RoombaDriveMethod(chassis, Slide, Pinch);

        // Use slide positions
        int slideInitial = Slide.getCurrentPosition();
        Slide.setTargetPosition(slideInitial);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initalize recording
        // initOCVRecording();

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
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (gamepad1.a && !gamepad1.start && !gamepad2.start) {
                speed = RoombaConstants.LOW_SPEED;
                releasedA1 = false;
            } else if (!releasedA1) {
                releasedA1 = true;
            }

            if (gamepad1.b && !gamepad1.start && !gamepad2.start) {
                speed = RoombaConstants.HIGH_SPEED;
                releasedB1 = false;
            } else if (!releasedB1) {
                releasedB1 = true;
            }

            if (gamepad1.y) {
//                TrajectorySequence traj = chassis.trajectorySequenceBuilder(chassis.getPoseEstimate())
//                        .addDisplacementMarker(() -> {
//                            Slide.setTargetPosition(slideInitial + 285);
//                            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            Slide.setPower(0.7);
//                        })
//                        .lineToSplineHeading(new Pose2d(64, 12, toRadians(0)))
//                        .build();
//                chassis.followTrajectorySequenceAsync(traj);
                releasedY1 = false;
            } else if (!releasedY1) {
                releasedY1 = true;
            }

            if (gamepad1.dpad_up) {
                if (releasedDU1) {
                    increaseSpeed(RoombaConstants.SPEED_INCREMENT);
                    releasedDU1 = false;
                }
            } else if (!releasedDU1) {
                releasedDU1 = true;
            }

            if (gamepad1.dpad_down) {
                if (releasedDD1) {
                    decreaseSpeed(RoombaConstants.SPEED_INCREMENT);
                    releasedDD1 = false;
                }
            } else if (!releasedDD1){
                releasedDD1 = true;
            }

            if (gamepad1.right_bumper) {
                if (releasedRB1) {
                    driveMethod.setPinched(!(Pinch.getPosition() > RoombaConstants.PINCH_MIDPOINT));
                    releasedRB1 = false;
                }
            } else if (!releasedRB1) {
                releasedRB1 = true;
            }

            if (gamepad2.a && !gamepad1.start && !gamepad2.start) {
                if (releasedA2) {
                    driveMethod.slideTo(slideInitial, 0.9);
                    releasedA2 = false;
                }
            } else if (!releasedA2) {
                releasedA2 = true;
            }

            if (gamepad2.b && !gamepad1.start && !gamepad2.start) {
                if (releasedB2) {
                    driveMethod.slideTo(slideInitial + RoombaConstants.SL_LOW, 0.9);
                    releasedB2 = false;
                }
            } else if (!releasedB2) {
                releasedB2 = true;
            }

            if (gamepad2.x) {
                if (releasedX2) {
                    driveMethod.slideTo(slideInitial + RoombaConstants.SL_MEDIUM, 0.9);
                    releasedX2 = false;
                }
            } else if (!releasedX2) {
                releasedX2 = true;
            }

            if (gamepad2.y) {
                if (releasedY2) {
                    driveMethod.slideTo(slideInitial + RoombaConstants.SL_HIGH, 0.9);
                    releasedY2 = false;
                }
            } else if (!releasedY2) {
                releasedY2 = true;
            }

            if (gamepad2.dpad_up) {
                if (releasedDU2) {
                        driveMethod.slideTo(Slide.getCurrentPosition() + 105, 1.0);
                    releasedDU2 = false;
                }
            } else if (!releasedDU2){
                releasedDU2 = true;
            }

            if (gamepad2.dpad_down) {
                if (releasedDD2) {
                    driveMethod.slideTo(Slide.getCurrentPosition() - 105, 1.0);
                    releasedDD2 = false;
                }
            } else if (!releasedDD2) {
                releasedDD2 = true;
            }

            if (gamepad2.left_bumper) {
                if (releasedLB2) {
                    slideInitial = Slide.getCurrentPosition();
                    releasedLB2 = false;
                }
            } else if (!releasedLB2) {
                releasedLB2 = true;
            }

            if (gamepad2.right_bumper) {
                if (releasedRB2) {
                    driveMethod.setPinched(!(Pinch.getPosition() > RoombaConstants.PINCH_MIDPOINT));
                    releasedRB2 = false;
                }
            } else if (!releasedRB2) {
                releasedRB2 = true;
            }

            chassis.setMotorPowers(
                    Range.clip(speed * (drive + rotate - strafe), -1.0, 1.0),
                    Range.clip(speed * (drive + rotate + strafe), -1.0, 1.0),
                    Range.clip(speed * (drive - rotate - strafe), -1.0, 1.0),
                    Range.clip(speed * (drive - rotate + strafe), -1.0, 1.0)
            );

            // telemetry.addData("Recording FPS", String.format("%.2f", Camera.getFps()));
            telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
            telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
            telemetry.addLine("Pinch: " + Pinch.getPosition());
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("Speed:", speed);
            telemetry.update();
        }
        // Camera.stopRecordingPipeline();
    }

    private void initOCVRecording() {
        this.Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Camera = OpenCvCameraFactory.getInstance().createWebcam(Webcam, cameraMonitorViewId);

        Camera.setPipeline(new RecordingPipeline(this.Camera));
        Camera.setViewportRenderingPolicy(OpenCvWebcam.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        Camera.setViewportRenderer(OpenCvWebcam.ViewportRenderer.GPU_ACCELERATED);
        Camera.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Camera.startStreaming(RoombaConstants.OCV_RECORDING_WIDTH, RoombaConstants.OCV_RECORDING_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Could not start recording.");
                telemetry.update();
            }
        });
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
