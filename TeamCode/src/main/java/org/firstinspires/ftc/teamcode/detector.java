
package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    //public detector(Telemetry t) {telemetry = t; }

    @Override
    public Mat processFrame(Mat input){
        //Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Scalar lowHSV = new Scalar (19,65,37);
//        Scalar highHSV = new Scalar (31, 100, 76);

//        Core.inRange(mat, lowHSV, highHSV, mat);

        return input;
    }
}
