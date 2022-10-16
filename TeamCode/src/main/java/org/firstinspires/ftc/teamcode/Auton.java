package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton", group="auto")
public class Auton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        Robot bot = new Robot(hardwareMap, this);
        bot.initOpenCV();
//        telemetry.addData("o3", DetectorPipeline.orangeCnt);
//        telemetry.addData("o3", DetectorPipeline.purpleCnt);
//        telemetry.addData("o3", DetectorPipeline.greenCnt);
        telemetry.addData("sleeveColor", DetectorPipeline.sleeveColor);
//        telemetry.addData("c1", detector.pixelColor[0]);
//        telemetry.addData("c2", detector.pixelColor[1]);
//        telemetry.addData("c3", detector.pixelColor[2]);
//        telemetry.addData("c4", detector.pixelColor[3]);

        telemetry.update();
        waitForStart();



    }
}
