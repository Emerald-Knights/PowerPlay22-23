package org.firstinspires.ftc.teamcode.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class LocalizationPipeline extends OpenCvPipeline {
    public Mat processFrame(Mat input) {
        Core.flip(input, input, -1);
        Mat output = input.clone();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);
        Core.inRange(output, new Scalar(0, 0, 0), new Scalar(360, 100, 100), output); //cb:100
        return output;
    }
}
