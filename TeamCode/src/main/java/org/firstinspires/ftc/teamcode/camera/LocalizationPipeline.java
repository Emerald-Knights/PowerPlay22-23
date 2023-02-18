package org.firstinspires.ftc.teamcode.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class LocalizationPipeline extends OpenCvPipeline {
    public Mat processFrame(Mat input) {
        Mat mat = input.clone();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Mat overlay = new Mat();
        Core.inRange(mat, new Scalar(100, 100, 100), new Scalar(140, 255, 255), overlay); //blue colors
        Mat masked = new Mat();
        Core.bitwise_and(mat, mat, masked, overlay);
        Mat cropped = new Mat();
        Imgproc.cvtColor(masked, cropped, Imgproc.COLOR_HSV2RGB);

        Mat edges = new Mat();
        Imgproc.Canny(cropped, edges, 150, 200);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        for(int i = 0; i < boundRect.length; i++) {
            Imgproc.rectangle(input, boundRect[i], new Scalar(62, 255, 127));
        }

        //clean up memory
        //edges.copyTo(input);
        cropped.release();
        mat.release();
        overlay.release();
        masked.release();

        return input;
    }
}
