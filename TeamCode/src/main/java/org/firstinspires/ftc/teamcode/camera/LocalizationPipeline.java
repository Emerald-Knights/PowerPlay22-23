package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class LocalizationPipeline extends OpenCvPipeline {

    LinearOpMode linearOpMode;

    public double pixelsOffCenter;
    public double angle;
    static public final double RAD_PER_PIXEL = 0.00253807106; //inverse of slope of graph

    int scanHeight = 10;
    int yDev = 100;

    public LocalizationPipeline(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public LocalizationPipeline() {
        //heyyyy
    }

    public Mat processFrame(Mat input) {
        Size imageSize = input.size();

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

        Rect crosshair2 = new Rect(new Point(0, yDev), new Point(imageSize.width, yDev+scanHeight));
        Imgproc.rectangle(cropped, crosshair2, new Scalar(4,233,78),3,8);

        //i = x
        //j = y
        int sumX = 0;
        int cnt=1;
        for(int i = 0; i < imageSize.width; i++) {
            for (int j = yDev; j < scanHeight+yDev; j++) {
                double[] pixelColor = cropped.get(j,i);
                if(pixelColor != null && pixelColor[0] > 50) { //just to make sure it exists and it's not black
                    sumX += i;
                    cnt++;
                }
            }
        }
        int averageX = sumX/cnt;
        Imgproc.line(cropped, new Point(averageX, 0), new Point(averageX, imageSize.height), new Scalar(100, 100, 100), 5);

        pixelsOffCenter = averageX-(imageSize.width/2);
        angle = pixelsOffCenter * RAD_PER_PIXEL;

        //clean up memory
        cropped.copyTo(input);
        cropped.release();
        mat.release();
        overlay.release();
        masked.release();
        edges.release();

        return input;
    }
}
