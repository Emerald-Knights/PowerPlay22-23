package org.firstinspires.ftc.teamcode.EKopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="AutonParkWithTime", group="auto")
public class AutonParkTimeBased extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        Robot wucru = new Robot(hardwareMap, this);

        wucru.initOpenCV();
        waitForStart();
        wucru.resetEncoders();
//        telemetry.addData("o3", DetectorPipeline.orangeCnt);
//        telemetry.addData("o3", DetectorPipeline.purpleCnt);
//        telemetry.addData("o3", DetectorPipeline.greenCnt);
//        telemetry.addData(AutonParkTimeBased"sleeveColor", DetectorPipeline.sleeveColor);
//        telemetry.addData("c1", detector.pixelColor[0]);
//        telemetry.addData("c2", detector.pixelColor[1]);
//        telemetry.addData("c3", detector.pixelColor[2]);
//        telemetry.addData("c4", detector.pixelColor[3]);
        telemetry.update();
        int sleeveColor = DetectorPipeline.sleeveColor;

        wucru.strafeWtime(-1,0.2,0.7);
        sleep(100);
        wucru.straightWtime(1,0.1,0.8);

/*
        if (sleeveColor == 1){
            wucru.strafeWtime(-1,0.4,0.8);
            wucru.resetEncoders();
            wucru.straightWtime(1,0.4,0.8);
        }
        if(sleeveColor == 2){
            wucru.straightWtime(1, 0.4, 0.8);
        }
        if (sleeveColor == 3){
            wucru.strafeWtime(1,0.4,0.8);
            wucru.resetEncoders();
            wucru.straightWtime(1, 0.4, 0.8);
        }

*/

    }
}
