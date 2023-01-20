package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.Color;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class ColorDetectionPipeline extends OpenCvPipeline {
    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }
                                                                        //300
    private static final Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(250, 50);
    public static int REGION_WIDTH = 125; // 225 in striker
    public static int REGION_HEIGHT = 200;
                                    //370 in striker

    private final Scalar
            // PUMPKIN = new Scalar(255, 100, 0),
            // EYES    = new Scalar(0, 150, 0),
            // BAT     = new Scalar(255, 0, 255);
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    Point topLeftPt = new Point(SLEEVE_TOPLEFT_ANCHOR_POINT.x, SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point bottomRightPt = new Point(SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        Mat areaMat = input.submat(new Rect(topLeftPt, bottomRightPt));
        Scalar sumColors = Core.sumElems(areaMat);

        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        if (sumColors.val[0] == minColor) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    topLeftPt,
                    bottomRightPt,
                    CYAN,
                    2
            );
        } else if (sumColors.val[1] == minColor) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    topLeftPt,
                    bottomRightPt,
                    MAGENTA,
                    2
            );
        } else {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    topLeftPt,
                    bottomRightPt,
                    YELLOW,
                    2
            );
        }
        areaMat.release();
        return input;
    }

    public ParkingPosition getPosition() {
        return position;
    }
}