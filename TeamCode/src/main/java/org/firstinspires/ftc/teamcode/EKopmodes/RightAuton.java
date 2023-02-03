package org.firstinspires.ftc.teamcode.EKopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="RightAuton", group="auto")
public class RightAuton extends LinearOpMode {

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
        //temporarily set sleevecolor because openCV no work at olive's hous
        int sleeveColor = 1;

        wucru.resetEncoders();

        //go forward and turn to junction
        wucru.straight(1,10,0.2);
        wucru.moveSlide(0.6,1);
        wucru.turnTo(Math.PI/4, 0.2);

        //move to junction, drop cone, retreat
        wucru.resetEncoders();
        sleep(200);
        wucru.straight(1,2.5, 0.15);
        wucru.moveSlide(-0.2,0.5);
        wucru.resetEncoders();
        sleep(100);
        wucru.straight(-1, 2.5, 0.15);
        wucru.resetEncoders();
        sleep(1000);

        //go to stack
        wucru.turnTo(0.1, 0.16);
        wucru.moveSlide(-0.4, 1.2);
        wucru.resetEncoders();
        wucru.straight(1,9,0.15);
        sleep(500);
        wucru.turnTo(1.86,-0.15);
        sleep(500);
        wucru.resetEncoders();
        wucru.straightWtime(1, 0.13, 2);
        sleep(500);

        //go to high junction
        wucru.resetEncoders();
        wucru.straight(-1,13, 0.2);
        sleep(500);
        telemetry.addData("angle", wucru.imu.getAngularOrientation());
        telemetry.update();
        while(wucru.imu.getAngularOrientation().firstAngle > 0){
            wucru.rightFront.setPower(0.12);
            wucru.rightBack.setPower(0.12);
            wucru.leftFront.setPower(-0.06);
            wucru.leftBack.setPower(-0.12);
        }
        wucru.leftBack.setPower(0);
        wucru.leftFront.setPower(0);
        wucru.rightBack.setPower(0);
        wucru.rightFront.setPower(0);
//        wucru.turnTo(0,0.12);
//        wucru.moveSlide(0.6, 1.8);
//        wucru.resetEncoders();
//        wucru.straight(1,5.5,0.12);
//        sleep(500);
//
//
//        //retreat to park
//        wucru.moveSlide(-0.2, 0.3);
//        wucru.resetEncoders();
//        wucru.straight(-1, 2.5,0.15);
//        wucru.resetEncoders();
//        wucru.strafe(-1, 2.2, 0.2);
//
//        if(sleeveColor==1){
//            wucru.turnTo(0, 0.15);
//            wucru.resetEncoders();
//            wucru.strafe(-1, 7, 0.16);
//            wucru.moveSlide(-0.2, 5);
//        }
//
//        else if(sleeveColor==3){
//            wucru.turnTo(0, 0.15);
//            wucru.resetEncoders();
//            wucru.straight(1,3,0.15);
//            wucru.strafe(1, 7, 0.16);
//            wucru.moveSlide(-0.2, 5);
//        }
//
//        else{
//            wucru.moveSlide(-0.2, 5);
//        }

    }
}
