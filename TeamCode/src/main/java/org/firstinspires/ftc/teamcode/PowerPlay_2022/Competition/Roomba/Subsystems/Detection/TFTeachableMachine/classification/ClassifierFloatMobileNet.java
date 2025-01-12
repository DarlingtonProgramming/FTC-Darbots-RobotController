package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems.Detection.TFTeachableMachine.classification;

import android.app.Activity;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.tensorflow.lite.support.common.TensorOperator;
import org.tensorflow.lite.support.common.ops.NormalizeOp;

public class ClassifierFloatMobileNet extends Classifier {
    /** Float MobileNet requires additional normalization of the used input. */
    private static final float IMAGE_MEAN = 127.5f;

    private static final float IMAGE_STD = 127.5f;

    /**
     * Float model does not need dequantization in the post-processing. Setting mean and std as 0.0f
     * and 1.0f, repectively, to bypass the normalization.
     */
    private static final float PROBABILITY_MEAN = 0.0f;

    private static final float PROBABILITY_STD = 1.0f;

    /**
     * Initializes a {@code ClassifierFloatMobileNet}.
     *
     * @param activity
     */
    public ClassifierFloatMobileNet(Activity activity, Device device, int numThreads, String modelFileName, String labelFileName, Telemetry t)
            throws Exception {
        super(activity, device, numThreads, modelFileName, labelFileName, t);
    }

    @Override
    protected TensorOperator getPreprocessNormalizeOp() {
        return new NormalizeOp(IMAGE_MEAN, IMAGE_STD);
    }

    @Override
    protected TensorOperator getPostprocessNormalizeOp() {
        return new NormalizeOp(PROBABILITY_MEAN, PROBABILITY_STD);
    }
}
