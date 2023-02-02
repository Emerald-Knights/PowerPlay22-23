package org.firstinspires.ftc.teamcode.EKopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="LeftAuton", group="auto")
public class LeftAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        Robot wucru = new Robot(hardwareMap, this);

        wucru.initOpenCV();
        waitForStart();
        wucru.resetEncoders();
//        telemetry.addData("o3", DetectorPipeline.orangeCnt);
//        telemetry.addData("o3", DetectorPipeline.purpleCnt);
//        telemetry.addData("o3", DetectorPipeline.greenCnt);
//        telemetry.addData(AutonPark"sleeveColor", DetectorPipeline.sleeveColor);
//        telemetry.addData("c1", detector.pixelColor[0]);
//        telemetry.addData("c2", detector.pixelColor[1]);
//        telemetry.addData("c3", detector.pixelColor[2]);
//        telemetry.addData("c4", detector.pixelColor[3]);
        telemetry.update();
        int sleeveColor = DetectorPipeline.sleeveColor;

//        wucru.moveClaw();
//        wucru.moveClaw();

//        if (sleeveColor == 1){
            wucru.resetEncoders();
            wucru.straight(1,11,0.2);
            sleep(1000);
            wucru.moveSlide(0.6,2.5);
            wucru.turnTo(3*Math.PI/4, 0.2);
            wucru.resetEncoders();
            sleep(1000);
            wucru.straight(1,4, 0.15);

            wucru.moveSlide(-0.2,0.5);
            wucru.resetEncoders();
            sleep(1000);
            wucru.straight(-1, 4, 0.15);
            wucru.resetEncoders();
            sleep(1000);
            wucru.turnTo(0, 0.16);
            wucru.moveSlide(-0.4, 1.2);
            sleep(2000);
            wucru.moveSlide(-0.4, 0.6);
            sleep(2000);
            wucru.moveSlide(-0.2, 1);
            wucru.resetEncoders();
            wucru.straight(1,6,0.2);
            sleep(1000);
            wucru.turnTo(-Math.PI/2,-0.15);
            sleep(1000);
            wucru.resetEncoders();
            wucru.straightWtime(1, 0.1, 2);
            sleep(1000);
            wucru.resetEncoders();
            wucru.straightWtime(-1,0.1, 2);
//            wucru.strafe(1, 14,0.5);
//            wucru.moveSlide(0.5, 5);
//            wucru.straight(1,10,0.3);
//            wucru.moveClaw();
//            wucru.straight(-1,10,0.3);
//            wucru.moveSlide(-0.5, 5);


//            wucru.strafe(-1,54,0.4);
            /*
            og code
            wucru.strafe(-1,27,0.5);
            wucru.resetEncoders();
            wucru.straight(1,30,0.5);
             */
//        }
//        if(sleeveColor == 2){
//            wucru.resetEncoders();
//            wucru.straight(1,30,0.5);
//            wucru.strafe(1, 14,0.5);
//            wucru.moveSlide(0.5, 5);
//            wucru.straight(1,10,0.3);
//            wucru.moveClaw();
//            wucru.straight(-1,10,0.3);
//            wucru.moveSlide(-0.5, 5);
//
//
//            wucru.strafe(-1,27,0.4);
//
//            /*
//            wucru.straight(1, 28, 0.8);
//            */
//        }
//
//        if (sleeveColor == 3){
//            wucru.resetEncoders();
//            wucru.straight(1,60,0.5);
//            wucru.strafe(1, 14,0.5);
//            wucru.moveSlide(0.5, 5);
//            wucru.straight(1,10,0.3);
//            wucru.moveClaw();
//            wucru.straight(-1,10,0.3);
//            wucru.moveSlide(-0.5, 5);
//
//
//            wucru.strafe(1,14,0.4);
//            /*
//            wucru.strafe(1,27,0.8);
//            wucru.resetEncoders();
//            wucru.straight(1, 30, 0.8);
//             */
//        }
    }
}
