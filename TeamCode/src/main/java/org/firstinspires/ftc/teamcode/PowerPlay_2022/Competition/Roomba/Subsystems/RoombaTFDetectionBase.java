package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems;

import static org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants.AllianceType;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Settings.RoombaConstants;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.TFTeachableMachine.PPDetector;
import org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.TFTeachableMachine.classification.Classifier;

import java.util.List;

public class RoombaTFDetectionBase extends PPDetector {
    private String visionResult = null;
    public RoombaTFDetectionBase(AllianceType type, HardwareMap hardwareMap, Telemetry telemetry) throws Exception {
        super(Classifier.Model.FLOAT_EFFICIENTNET, type == AllianceType.BLUE ? RoombaConstants.BLUE_MODEL : RoombaConstants.RED_MODEL, RoombaConstants.LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
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
