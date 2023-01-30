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

        int sleeveColor = DetectorPipeline.sleeveColor;

//        wucru.moveClaw();
//        wucru.moveClaw();
        sleep(1000);
//        wucru.moveSlide(0.4,1.4);
        telemetry.addData("detected: ", sleeveColor);
        sleep(1000);
        if (sleeveColor == 1){
            telemetry.addLine("in 1 loop");
            wucru.strafeWtime(-1,0.2,0.8);
            sleep(1000);
            telemetry.addData("heading: ", wucru.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            wucru.turnTo(0, 0.08);
            sleep(1000);
            wucru.straightWtime(1,0.1,1);
        }
        else if(sleeveColor == 2){
            telemetry.addLine("in 2 loop");
            wucru.straightWtime(1, 0.1, 1.3);
        }
        else if (sleeveColor == 3){
            telemetry.addLine("in 3 loop");
            wucru.strafeWtime(1,0.2,0.8);
            sleep(1000);
            telemetry.addData("heading: ", wucru.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            wucru.turnTo(0, 0.08);
            sleep(1000);
            wucru.straightWtime(1,0.1,1.1);
        }



    }
}
