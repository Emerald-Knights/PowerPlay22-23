package org.firstinspires.ftc.teamcode.EKopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Auton", group="auto")
public class AutonPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        Robot wucru = new Robot(hardwareMap, this);

        wucru.initOpenCV();
        waitForStart();
        wucru.resetEncoders();
//        telemetry.addData("o3", DetectorPipeline.orangeCnt);
//        telemetry.addData("o3", DetectorPipeline.purpleCnt);
//        telemetry.addData("o3", DetectorPipeline.greenCnt);
        telemetry.addData("sleeveColor", DetectorPipeline.sleeveColor);
//        telemetry.addData("c1", detector.pixelColor[0]);
//        telemetry.addData("c2", detector.pixelColor[1]);
//        telemetry.addData("c3", detector.pixelColor[2]);
//        telemetry.addData("c4", detector.pixelColor[3]);
        telemetry.update();
        int sleeveColor = DetectorPipeline.sleeveColor;

        if (sleeveColor == 1){
            wucru.strafe(-1,27,0.8);
            wucru.resetEncoders();
            wucru.straight(1,30,0.8);
        }
        if(sleeveColor == 2){
            wucru.straight(1, 28, 0.8);
        }
        if (sleeveColor == 3){
            wucru.strafe(1,27,0.8);
            wucru.resetEncoders();
            wucru.straight(1, 30, 0.8);
        }



    }
}
