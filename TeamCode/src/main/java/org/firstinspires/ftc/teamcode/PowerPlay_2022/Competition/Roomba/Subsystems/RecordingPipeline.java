package org.firstinspires.ftc.teamcode.PowerPlay_2022.Competition.Roomba.Subsystems;

import java.text.SimpleDateFormat;
import java.util.Date;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

public class RecordingPipeline extends OpenCvPipeline {
    boolean toggleRecording = false;
    private OpenCvWebcam Camera;

    public RecordingPipeline(OpenCvWebcam Camera) {
        this.Camera = Camera;
    }

    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    @Override
    public void onViewportTapped() {
        toggleRecording = !toggleRecording;

        if (toggleRecording) {
            SimpleDateFormat formatter = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
            Date date = new Date();
            this.Camera.startRecordingPipeline(
                    new PipelineRecordingParameters.Builder()
                            .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps)
                            .setEncoder(PipelineRecordingParameters.Encoder.H264)
                            .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                            .setFrameRate(30)
                            .setPath("/sdcard/" + formatter.format(date) + ".mp4")
                            .build());
        } else {
            this.Camera.stopRecordingPipeline();
        }
    }
}