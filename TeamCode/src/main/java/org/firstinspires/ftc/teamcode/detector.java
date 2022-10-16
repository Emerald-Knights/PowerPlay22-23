
package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class detector extends OpenCvPipeline {
    Telemetry telemetry;

    double[] targetOrange = {220, 160, 120};
    //insert target purple and target green as well

    double pctColorError = 0.2;


    int p1 = 0;
    int g2 = 0;
    public static int o3 = 0;

    public static double[] pixelColor = {0,0,0,0};

    @Override
    public Mat processFrame(Mat input){
        Size imageSize = input.size();
        Mat output = input.clone();
//
//        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);
//        Scalar lowHSV = new Scalar (50,50,50);
//        Scalar highHSV = new Scalar (200, 200, 200);
//
//        Core.inRange(output, lowHSV, highHSV, output);
        Rect crosshair1 = new Rect(new Point(0, imageSize.height/2), new Point(imageSize.width, imageSize.height - 50));
        Imgproc.rectangle(output, crosshair1, new Scalar(4, 233, 78), 3, 8);

//
//        for(int i = (int)(imageSize.height); i < imageSize.height-50 ; i++){
//            for(int j = 0; j < imageSize.width; j++)
//            {

                pixelColor = input.get(10, 10);
//                if (pixelColor[0] < 200 && pixelColor[1] < 200 && pixelColor[2] < 200){
//                    o3++;
//
//                }
//
//                else{
//
//                }
//                double[] newColor = {50, 50, 50, pixelColor[3]};
//                output.put(i, j, newColor);
//            }
//        }

        return output;
    }
}
