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

        telemetry.update();
        int sleeveColor = 1;

        wucru.resetEncoders();

        //go forward and turn to junction
        wucru.straight(1,12,0.2);
        wucru.moveSlide(0.6,1.5);
        wucru.turnTo(-Math.PI/4, 0.2);

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
        wucru.turnTo(0, 0.16);
        wucru.moveSlide(-0.4, 1.2);
        wucru.resetEncoders();
        wucru.straight(1,5,0.15);
        sleep(500);
        wucru.turnTo(-Math.PI/2,-0.15);
        sleep(500);
        wucru.resetEncoders();
        wucru.straightWtime(1, 0.1, 2);
        sleep(500);

        //go to high junction
        wucru.resetEncoders();
        wucru.straight(-1,9, 0.2);
        sleep(500);
        wucru.turnTo(0.12,0.12);
        wucru.moveSlide(0.6, 1.8);
        wucru.resetEncoders();
        wucru.straight(1,5.5,0.12);
        sleep(500);


        //retreat to park
        wucru.moveSlide(-0.2, 0.3);
        wucru.resetEncoders();
        wucru.straight(-1, 2.5,0.15);
        wucru.resetEncoders();
        wucru.strafe(-1, 2.2, 0.2);

        if(sleeveColor==1){
            wucru.turnTo(0, 0.15);
            wucru.resetEncoders();
            wucru.strafe(-1, 7, 0.16);
            wucru.moveSlide(-0.2, 5);
        }

        else if(sleeveColor==3){
            wucru.turnTo(0, 0.15);
            wucru.resetEncoders();
            wucru.straight(1,3,0.15);
            wucru.strafe(1, 7, 0.16);
            wucru.moveSlide(-0.2, 5);
        }

        else{
            wucru.moveSlide(-0.2, 5);
        }

    }
}
