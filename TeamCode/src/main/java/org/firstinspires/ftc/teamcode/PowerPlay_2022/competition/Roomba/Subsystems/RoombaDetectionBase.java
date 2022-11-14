package org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Subsystems;

import static org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Settings.Roomba_Constants.AllianceType;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.competition.Roomba.Settings.Roomba_Constants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.detection.PPDetector;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.detection.classification.Classifier;

import java.util.List;

public class RoombaDetectionBase extends PPDetector {
    private String visionResult = null;
    public RoombaDetectionBase(AllianceType type, HardwareMap hardwareMap, Telemetry telemetry) throws Exception {
        super(Classifier.Model.FLOAT_EFFICIENTNET, type == AllianceType.BLUE ? Roomba_Constants.BLUE_MODEL : Roomba_Constants.RED_MODEL, Roomba_Constants.LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
    }
    public String getCurrentResult() {
        return visionResult;
    }
    public String recogWithConfidence(double confidenceLevel) {
        List<Classifier.Recognition> results = this.getLastResults();
        if (results != null && results.size() != 0) {
            for (Classifier.Recognition r : results) {
                if (r.getConfidence() > confidenceLevel) {
                    return visionResult = r.getTitle();
                }
            }
        }
        return null;
    }
}
