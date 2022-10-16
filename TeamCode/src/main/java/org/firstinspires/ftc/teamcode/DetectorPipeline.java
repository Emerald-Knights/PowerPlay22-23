
package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectorPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    //insert target purple and target green as well
    double[] targetOrange = {220, 160, 120};

    double pctColorError = 0.2;


    //the final value after comparison for which sleeve color is shown
    public static int sleeveColor;

    //the dimensions of the area that is scanned
    int scanWidth = 80;
    int scanHeight = 80;

    //how much the scanned area is translated
    int xDev = 0;
    int yDev = 0;

    //brightness increases the color threshold. If it is not detecting, try increasing brightness.
    int brightness = 0;

    public static double[] pixelColor = {0,0,0,0};

    @Override
    public Mat processFrame(Mat input){
        Size imageSize = input.size();
        Mat output = input.clone();
        int orangeCnt = 0;
        int greenCnt = 0;
        int purpleCnt = 0;
        Rect crosshair1 = new Rect(new Point(imageSize.width/2 + xDev + scanWidth, (imageSize.height)/2 + yDev - scanHeight), new Point((int)(imageSize.width/2) + xDev - scanWidth, imageSize.height/2 +yDev + scanHeight));
        Imgproc.rectangle(output, crosshair1, new Scalar(4,233,78),3,8);
        for(int i = (int)(imageSize.height)/2 + yDev - scanHeight; i < imageSize.height/2 +yDev + scanHeight ; i++){
            for(int j = (int)(imageSize.width/2) + xDev - scanWidth; j < imageSize.width/2 + xDev + scanWidth; j++)
            {
                pixelColor = input.get(i,j);
                //orange
                if (pixelColor[0] > 210 + brightness){
                    orangeCnt++;
                    double[] newColor = {250, 0, 0, pixelColor[3]};
                    output.put(i, j, newColor);
                }
                //green
                if (pixelColor[1] > 160 + brightness && pixelColor[0] < 180 + brightness){
                    greenCnt++;
                    double[] newColor = {0, 250, 0, pixelColor[3]};
                    output.put(i, j, newColor);
                }
                //purple
                if (pixelColor[2] > 120 + brightness && pixelColor[1] < 130){
                    purpleCnt++;
                    double[] newColor = {0, 0, 250, pixelColor[3]};
                    output.put(i, j, newColor);
                }
                //if it is white
                if (pixelColor[0] > 210 + brightness && pixelColor[1] > 210 + brightness && pixelColor[2] > 210 + brightness){
                    double[] newColor = {0, 0, 0, pixelColor[3]};
                    output.put(i, j, newColor);
                }
            }
        }
        if(orangeCnt > purpleCnt && orangeCnt > greenCnt){
            sleeveColor = 3;
        }
        if(purpleCnt > orangeCnt && purpleCnt > greenCnt){
            sleeveColor = 1;
        }
        if(greenCnt > orangeCnt && greenCnt > purpleCnt){
            sleeveColor = 2;
        }

        return output;
    }
}